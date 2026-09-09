#!/usr/bin/env python3
"""
UNDIP Mini ADV HMI Sync GUI — Modern Dark Dashboard
Sinkronisasi dua arah: GUI Python  ⇄  LCD STM32 (ILI9341 + XPT2046)

Protokol serial (115200 8N1):
  GUI   -> STM32 : GOTO:HOME | GOTO:CAMERA | GOTO:GPS | GOTO:ACTUATOR | GOTO:SPLASH
  GUI   -> STM32 : GET:STATE | PING
  STM32 -> GUI   : PAGE:HOME | PAGE:CAMERA | PAGE:GPS | PAGE:ACTUATOR | PAGE:SPLASH
             STM32 -> GUI   : ACK:PONG | ERR:<msg>
"""

import glob
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial belum terinstall. Jalankan: python3 -m pip install pyserial")
    raise

BAUD = 1000000
PAGES = ["SPLASH", "HOME", "CAMERA", "GPS", "ACTUATOR"]
PAGE_LABELS = {
    "SPLASH": "SPLASH",
    "HOME": "HOME",
    "CAMERA": "CAMERA",
    "GPS": "GPS",
    "ACTUATOR": "ACTUATOR",
}
PAGE_ICONS = {
    "SPLASH": "◉",
    "HOME": "⌂",
    "CAMERA": "▣",
    "GPS": "⌖",
    "ACTUATOR": "◉",
}

# ─────────────────────────────────────────────────────────────
# Color palette (dark dashboard)
# ─────────────────────────────────────────────────────────────
C_BG          = "#0d1117"
C_PANEL       = "#161b22"
C_PANEL_HI    = "#1f2630"
C_BORDER      = "#30363d"
C_PRIMARY     = "#58a6ff"
C_PRIMARY_HO  = "#79c0ff"
C_ACCENT      = "#d2a8ff"
C_GREEN       = "#3fb950"
C_GREEN_DIM   = "#2ea043"
C_AMBER       = "#d29922"
C_RED         = "#f85149"
C_TEXT        = "#e6edf3"
C_TEXT_DIM    = "#8b949e"
C_TEXT_MUTED  = "#6e7681"
C_WHITE       = "#ffffff"


class RoundedButton(tk.Canvas):
    """Canvas-based rounded rectangle button with hover/press/active states."""
    def __init__(self, master, text, icon="", command=None,
                 bg=C_PANEL, fg=C_TEXT, active_bg=C_PRIMARY, active_fg=C_WHITE,
                 radius=10, height=52, font=("Segoe UI", 11, "bold"), **kw):
        super().__init__(master, highlightthickness=0, bd=0, bg=C_PANEL, height=height, **kw)
        self._cmd = command
        self._text = text
        self._icon = icon
        self._bg = bg
        self._fg = fg
        self._active_bg = active_bg
        self._active_fg = active_fg
        self._radius = radius
        self._height = height
        self._font = font
        self._state = "normal"   # normal | hover | active | disabled
        self._selected = False
        self.bind("<Enter>", lambda e: self._set_state("hover"))
        self.bind("<Leave>", lambda e: self._set_state("normal"))
        self.bind("<ButtonPress-1>", lambda e: self._set_state("active"))
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Configure>", lambda e: self._draw())
        self._draw()

    def _set_state(self, state):
        if state == self._state:
            return
        self._state = state
        self._draw()

    def _on_release(self, event):
        if self._state == "active":
            self._set_state("hover" if self.winfo_containing(event.x_root, event.y_root) == self else "normal")
            if self._cmd:
                self._cmd()

    def _draw(self):
        self.delete("all")
        w = self.winfo_width() or 160
        h = self._height
        r = min(self._radius, h // 2)
        # colors per state
        if self._state == "disabled":
            fill = "#21262d"
            outline = C_BORDER
            txt_col = C_TEXT_MUTED
        elif self._state == "active":
            fill = self._active_bg
            outline = self._active_bg
            txt_col = self._active_fg
        elif self._state == "hover":
            fill = C_PANEL_HI
            outline = C_PRIMARY
            txt_col = self._active_fg
        else:
            fill = self._active_bg if self._selected else self._bg
            outline = self._active_bg if self._selected else C_BORDER
            txt_col = self._active_fg if self._selected else self._fg
        # rounded rect
        self.create_rounded_rect(0, 0, w, h, r, fill=fill, outline=outline, width=1.5)
        # text + icon
        label = f"{self._icon}  {self._text}" if self._icon else self._text
        self.create_text(w // 2, h // 2, text=label, fill=txt_col, font=self._font, anchor="center")

    def create_rounded_rect(self, x1, y1, x2, y2, r, **kw):
        """Draw rounded rectangle on canvas."""
        points = [
            x1+r, y1,
            x2-r, y1,
            x2, y1,
            x2, y1+r,
            x2, y2-r,
            x2, y2,
            x2-r, y2,
            x1+r, y2,
            x1, y2,
            x1, y2-r,
            x1, y1+r,
            x1, y1,
        ]
        return self.create_polygon(points, smooth=True, **kw)

    def set_active(self, is_active: bool):
        self._selected = is_active
        self._draw()

    def set_enabled(self, enabled: bool):
        self._state = "normal" if enabled else "disabled"
        self._draw()


class HmiSyncGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UNDIP Mini ADV — HMI Sync")
        # counter GNOME fractional/high-DPI scaling
        self.tk.call("tk", "scaling", 1.0)
        self.geometry("1000x700")
        self.minsize(860, 600)
        self.configure(bg=C_BG)

        # serial state
        self.ser = None
        self.reader_thread = None
        self.stop_reader = threading.Event()
        self.rx_queue = queue.Queue()
        self.current_page = "DISCONNECTED"
        self.last_rx_time = 0.0

        self._build_style()
        self._build_ui()
        self.refresh_ports()
        self.after(80, self.process_rx_queue)
        self.after(1000, self.periodic_refresh)
        self.after(350, self.auto_connect)

    # ─────────────────────────────────────────────────────────
    # UI Construction
    # ─────────────────────────────────────────────────────────
    def _build_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=C_BG)
        style.configure("TLabelframe", background=C_PANEL, bordercolor=C_BORDER,
                        lightcolor=C_BORDER, darkcolor=C_BORDER)
        style.configure("TLabelframe.Label", background=C_PANEL, foreground=C_TEXT, font=("Segoe UI", 10, "bold"))
        style.configure("TLabel", background=C_PANEL, foreground=C_TEXT)
        style.configure("TCombobox", fieldbackground=C_PANEL_HI, background=C_PANEL_HI,
                        foreground=C_TEXT, arrowcolor=C_TEXT, bordercolor=C_BORDER)
        style.map("TCombobox", fieldbackground=[("readonly", C_PANEL_HI)],
                  selectbackground=[("readonly", C_PRIMARY)],
                  selectforeground=[("readonly", C_WHITE)])
        style.configure("TButton", background=C_PANEL_HI, foreground=C_TEXT,
                        bordercolor=C_BORDER, focusthickness=0, focuscolor=C_PRIMARY,
                        padding=8, font=("Segoe UI", 9))
        style.map("TButton",
                  background=[("active", C_PRIMARY), ("disabled", "#21262d")],
                  foreground=[("active", C_WHITE), ("disabled", C_TEXT_MUTED)],
                  bordercolor=[("active", C_PRIMARY)])

        # custom styles
        style.configure("Title.TLabel", background=C_BG, foreground=C_TEXT, font=("Segoe UI", 20, "bold"))
        style.configure("Subtitle.TLabel", background=C_BG, foreground=C_TEXT_DIM, font=("Segoe UI", 10))
        style.configure("StatusOK.TLabel", background=C_PANEL, foreground=C_GREEN, font=("Segoe UI", 11, "bold"))
        style.configure("StatusErr.TLabel", background=C_PANEL, foreground=C_RED, font=("Segoe UI", 11, "bold"))
        style.configure("StatusWarn.TLabel", background=C_PANEL, foreground=C_AMBER, font=("Segoe UI", 11, "bold"))
        style.configure("Value.TLabel", background=C_PANEL, foreground=C_TEXT, font=("Segoe UI", 11))
        style.configure("Mono.TLabel", background=C_PANEL, foreground=C_TEXT_DIM, font=("Consolas", 9))

    def _build_ui(self):
        # main container with padding
        main = ttk.Frame(self, padding=16)
        main.pack(fill=tk.BOTH, expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(3, weight=1)

        # ── Header ────────────────────────────────────────────
        header = ttk.Frame(main)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="UNDIP MINI ADV", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="HMI Synchronization Dashboard", style="Subtitle.TLabel").grid(row=1, column=0, sticky="w")
        # right side: connection status pill
        self.conn_pill = ttk.Frame(header, style="TFrame")
        self.conn_pill.grid(row=0, column=2, rowspan=2, sticky="e")
        self._conn_dot = tk.Canvas(self.conn_pill, width=12, height=12, highlightthickness=0, bg=C_BG)
        self._conn_dot.pack(side=tk.LEFT, padx=(0, 6))
        self._conn_dot.create_oval(2, 2, 10, 10, fill="#21262d", outline=C_BORDER, width=1.5, tags="dot")
        self.conn_text = ttk.Label(self.conn_pill, text="DISCONNECTED", style="StatusErr.TLabel")
        self.conn_text.pack(side=tk.LEFT)

        # ── Connection Panel ──────────────────────────────────
        conn_frame = ttk.Labelframe(main, text="  Serial Connection  ", padding=12)
        conn_frame.grid(row=1, column=0, sticky="ew", pady=8)
        conn_frame.columnconfigure(1, weight=1)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w", padx=(0, 10))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=38, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(0, 12))

        btn_frame = ttk.Frame(conn_frame)
        btn_frame.grid(row=0, column=2, sticky="e")
        self.refresh_btn = ttk.Button(btn_frame, text="↻ Refresh", width=12, command=self.refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=4)
        self.connect_btn = ttk.Button(btn_frame, text="Connect", width=14, command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=4)
        self.ping_btn = ttk.Button(btn_frame, text="Ping", width=10, command=lambda: self.send_line("PING"), state=tk.DISABLED)
        self.ping_btn.pack(side=tk.LEFT, padx=4)

        # connection info row
        info_row = ttk.Frame(conn_frame)
        info_row.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        info_row.columnconfigure(1, weight=1)
        info_row.columnconfigure(3, weight=1)
        info_row.columnconfigure(5, weight=1)
        ttk.Label(info_row, text="Baud:").grid(row=0, column=0, sticky="w")
        ttk.Label(info_row, text="115200", style="Value.TLabel").grid(row=0, column=1, sticky="w")
        ttk.Label(info_row, text="Data:").grid(row=0, column=2, sticky="w", padx=(24, 0))
        ttk.Label(info_row, text="8N1", style="Value.TLabel").grid(row=0, column=3, sticky="w")
        ttk.Label(info_row, text="Flow:").grid(row=0, column=4, sticky="w", padx=(24, 0))
        ttk.Label(info_row, text="None", style="Value.TLabel").grid(row=0, column=5, sticky="w")

        # ── Active Page Panel ─────────────────────────────────
        page_frame = ttk.Labelframe(main, text="  Active Page  ", padding=12)
        page_frame.grid(row=2, column=0, sticky="ew", pady=8)
        page_frame.columnconfigure(1, weight=1)

        ttk.Label(page_frame, text="Current:").grid(row=0, column=0, sticky="w", padx=(0, 12))
        self.page_badge = ttk.Frame(page_frame, style="TFrame")
        self.page_badge.grid(row=0, column=1, sticky="w")
        self.page_icon_lbl = ttk.Label(self.page_badge, text="◉", style="Value.TLabel", font=("Segoe UI", 20))
        self.page_icon_lbl.pack(side=tk.LEFT, padx=(0, 8))
        self.page_name_lbl = ttk.Label(self.page_badge, text="DISCONNECTED", style="StatusErr.TLabel", font=("Segoe UI", 14, "bold"))
        self.page_name_lbl.pack(side=tk.LEFT)

        self.sync_status = ttk.Label(page_frame, text="Menunggu koneksi...", style="Mono.TLabel")
        self.sync_status.grid(row=0, column=2, sticky="e")

        # ── Page Control Buttons ──────────────────────────────
        ctrl_frame = ttk.Labelframe(main, text="  Page Control  ", padding=12)
        ctrl_frame.grid(row=3, column=0, sticky="nsew", pady=8)
        for i in range(5):
            ctrl_frame.columnconfigure(i, weight=1)

        self.page_buttons = {}
        for i, page in enumerate(PAGES):
            btn = RoundedButton(
                ctrl_frame,
                text=PAGE_LABELS[page],
                icon=PAGE_ICONS[page],
                command=lambda p=page: self.goto_page(p),
                bg=C_PANEL,
                fg=C_TEXT,
                active_bg=C_PRIMARY,
                active_fg=C_WHITE,
                radius=12,
                height=60,
                font=("Segoe UI", 11, "bold"),
            )
            btn.grid(row=0, column=i, padx=8, pady=8, sticky="ew")
            btn.set_enabled(False)
            self.page_buttons[page] = btn

        # ── Log Panel ─────────────────────────────────────────
        log_frame = ttk.Labelframe(main, text="  Serial Log  ", padding=8)
        log_frame.grid(row=4, column=0, sticky="nsew", pady=(8, 0))
        main.rowconfigure(4, weight=1)
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self.log = tk.Text(
            log_frame,
            wrap=tk.WORD,
            bg="#0d1117",
            fg="#c9d1d9",
            insertbackground=C_PRIMARY,
            font=("JetBrains Mono", 9),
            highlightthickness=0,
            bd=0,
        )
        self.log.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scroll.set)
        # log tags
        self.log.tag_config("gui", foreground=C_PRIMARY)
        self.log.tag_config("stm", foreground=C_GREEN)
        self.log.tag_config("err", foreground=C_RED)
        self.log.tag_config("warn", foreground=C_AMBER)
        self.log.tag_config("dim", foreground=C_TEXT_MUTED)

    # ─────────────────────────────────────────────────────────
    # Serial / Port handling
    # ─────────────────────────────────────────────────────────
    def refresh_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append(f"{p.device}  -  {p.description}")
        known = {item.split()[0] for item in ports}
        for dev in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
            if dev not in known:
                ports.append(dev)
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def selected_port(self):
        raw = self.port_var.get().strip()
        return raw.split()[0] if raw else ""

    def auto_connect(self):
        if not self.ser and self.selected_port():
            self.connect()

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.selected_port()
        if not port:
            messagebox.showwarning("Port kosong", "Pilih port serial dulu. Biasanya /dev/ttyACM0.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=0.5)
            time.sleep(1.8)
            self.ser.reset_input_buffer()
            self.stop_reader.clear()
            self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
            self.reader_thread.start()
            self._set_connected(True, port)
            self.append_log(f"[GUI] Connected to {port}", "gui")
            self.send_line("PING")
            self.send_line("GET:STATE")
        except Exception as e:
            self.ser = None
            messagebox.showerror("Gagal connect", str(e))
            self.append_log(f"[ERR] Connect failed: {e}", "err")

    def _set_connected(self, ok: bool, port: str = ""):
        if ok:
            self._conn_dot.itemconfig("dot", fill=C_GREEN, outline=C_GREEN)
            self.conn_text.configure(text=f"CONNECTED  •  {port}", style="StatusOK.TLabel")
            self.connect_btn.configure(text="Disconnect")
            self.ping_btn.configure(state=tk.NORMAL)
            for b in self.page_buttons.values():
                b.set_enabled(True)
            self.sync_status.configure(text="LCD ↔ GUI aktif")
        else:
            self._conn_dot.itemconfig("dot", fill="#21262d", outline=C_BORDER)
            self.conn_text.configure(text="DISCONNECTED", style="StatusErr.TLabel")
            self.connect_btn.configure(text="Connect")
            self.ping_btn.configure(state=tk.DISABLED)
            for b in self.page_buttons.values():
                b.set_enabled(False)
            self._set_page("DISCONNECTED")
            self.sync_status.configure(text="Menunggu koneksi...")

    def disconnect(self):
        self.stop_reader.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._set_connected(False)

    # ─────────────────────────────────────────────────────────
    # Serial reader thread
    # ─────────────────────────────────────────────────────────
    def reader_loop(self):
        while not self.stop_reader.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    break
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    self.rx_queue.put(line)
            except Exception as e:
                self.rx_queue.put(f"__ERROR__:{e}")
                break

    def process_rx_queue(self):
        try:
            while True:
                line = self.rx_queue.get_nowait()
                if line.startswith("__ERROR__:"):
                    self.append_log(f"[ERR] Serial reader: {line[10:]}", "err")
                    self.disconnect()
                    break
                self.handle_rx_line(line)
        except queue.Empty:
            pass
        self.after(80, self.process_rx_queue)

    def handle_rx_line(self, line):
        self.last_rx_time = time.time()
        if line.startswith("PAGE:"):
            page = line.split(":", 1)[1].strip().upper()
            if page == "DASHBOARD":
                page = "HOME"
            self._set_page(page)
            self.append_log(f"[STM32] {line}", "stm")
        elif line == "ACK:PONG":
            self.sync_status.configure(text="Ping OK — koneksi aktif")
            self.append_log(f"[STM32] {line}", "stm")
        elif line.startswith("ERR:"):
            self.sync_status.configure(text=line)
            self.append_log(f"[STM32] {line}", "err")
        else:
            self.append_log(f"[STM32] {line}", "dim")

    def _set_page(self, page: str):
        self.current_page = page
        label = PAGE_LABELS.get(page, page)
        icon = PAGE_ICONS.get(page, "◉")
        if page == "DISCONNECTED":
            self.page_name_lbl.configure(text="DISCONNECTED", style="StatusErr.TLabel")
            self.page_icon_lbl.configure(text="✕", foreground=C_RED)
        else:
            self.page_name_lbl.configure(text=label, style="StatusOK.TLabel")
            self.page_icon_lbl.configure(text=icon, foreground=C_PRIMARY)
        # highlight active button
        for p, btn in self.page_buttons.items():
            btn.set_active(p == page)

    def goto_page(self, page: str):
        self._set_page(page)  # instant GUI feedback
        self.send_line(f"GOTO:{page}")

    def send_line(self, line: str):
        if not self.ser or not self.ser.is_open:
            self.append_log(f"[GUI] Not sent, not connected: {line}", "warn")
            return
        try:
            self.ser.write((line + "\n").encode("utf-8"))
            self.ser.flush()
            self.append_log(f"[GUI] {line}", "gui")
        except Exception as e:
            self.append_log(f"[ERR] Send failed: {e}", "err")
            self.disconnect()

    def periodic_refresh(self):
        if self.ser and self.ser.is_open:
            self.send_line("GET:STATE")
        else:
            self.refresh_ports()
        self.after(3000, self.periodic_refresh)

    def append_log(self, text: str, tag: str = "dim"):
        ts = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"{ts} ", "dim")
        self.log.insert(tk.END, f"{text}\n", tag)
        self.log.see(tk.END)

    def destroy(self):
        self.disconnect()
        super().destroy()


if __name__ == "__main__":
    app = HmiSyncGui()
    app.mainloop()