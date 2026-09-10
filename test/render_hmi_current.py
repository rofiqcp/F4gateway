#!/usr/bin/env python3
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "test" / "previews_current"
OUT.mkdir(exist_ok=True)
W, H = 320, 240
BG = (9, 18, 30)
PANEL = (16, 30, 47)
PANEL2 = (27, 42, 60)
CARD = (245, 237, 229)
INK = (10, 25, 42)
TEXT = (246, 246, 244)
DIM = (178, 187, 198)
ACCENT = (10, 199, 229)
READY = (47, 198, 63)
FAULT = (255, 59, 48)
BORDER = (62, 82, 105)

FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
def font(size, bold=False):
    return ImageFont.truetype(BOLD_PATH if bold else FONT_PATH, size)
F8, F10, F12, F14 = font(8), font(10), font(12, True), font(14, True)
def label(draw, xy, text, f=F10, fill=TEXT, anchor="la"):
    draw.text(xy, text, font=f, fill=fill, anchor=anchor)

def round_box(draw, box, fill, outline=BORDER, radius=7):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=1)

def topbar(draw, title):
    draw.rectangle((0, 0, W, 29), fill=BG)
    label(draw, (12, 15), "<", F14, ACCENT, "mm")
    label(draw, (36, 15), title, F10, TEXT, "lm")
    x = 201
    for code in ("E", "P", "N", "S"):
        draw.ellipse((x, 11, x + 6, 17), fill=READY)
        label(draw, (x + 10, 14), code, F8, DIM, "lm")
        x += 30

def carousel(draw, selected, count):
    y = 188
    round_box(draw, (4, y, 80, 235), PANEL)
    round_box(draw, (86, y, 234, 235), PANEL2)
    round_box(draw, (240, y, 316, 235), PANEL)
    label(draw, (42, 212), "<", F14, TEXT, "mm")
    label(draw, (160, 212), f"{selected + 1} / {count}", F12, ACCENT, "mm")
    label(draw, (278, 212), ">", F14, TEXT, "mm")

def root_page(title, items, selected=0):
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    topbar(draw, title)
    for slot, item in enumerate(items[:3]):
        x = 6 + slot * 104
        round_box(draw, (x, 42, x + 99, 173), PANEL2 if slot == selected else PANEL,
                  ACCENT if slot == selected else BORDER)
        label(draw, (x + 50, 83), item, F10, ACCENT if slot == selected else TEXT, "mm")
        label(draw, (x + 50, 117), "READY", F8, READY, "mm")
        label(draw, (x + 50, 151), str(slot + 1), F8, DIM, "mm")
    carousel(draw, selected, len(items))
    return image
def overview_page():
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    topbar(draw, "OVERVIEW")
    round_box(draw, (6, 36, 314, 82), PANEL)
    label(draw, (16, 50), "AUTO | READY     0.0 km/h", F10, TEXT)
    label(draw, (16, 70), "3D FIX/17 SAT | IDLE > NO TARGET", F8, DIM)
    for i, item in enumerate(("ESC", "PERCEPTION", "NAVIGATION")):
        x = 6 + i * 104
        round_box(draw, (x, 89, x + 99, 182), CARD, ACCENT if i == 0 else BORDER)
        label(draw, (x + 50, 124), item, F10, INK, "mm")
        label(draw, (x + 50, 163), "READY", F8, READY, "mm")
    carousel(draw, 0, 4)
    return image

def manual_page():
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    topbar(draw, "MANUAL TEST")
    buttons = [
        ((108, 40, 212, 90), "FORWARD", ACCENT),
        ((6, 96, 100, 166), "LEFT", ACCENT),
        ((108, 96, 212, 166), "STOP", FAULT),
        ((220, 96, 314, 166), "RIGHT", ACCENT),
        ((108, 172, 212, 224), "REVERSE", ACCENT),
    ]
    for box, text, color in buttons:
        round_box(draw, box, PANEL2 if text != "STOP" else FAULT, color)
        label(draw, ((box[0] + box[2]) // 2, (box[1] + box[3]) // 2), text, F10,
              TEXT, "mm")
    label(draw, (160, 232), "HOLD 0.6s  |  RELEASE = STOP", F8, (242, 181, 56), "mm")
    return image
def main():
    pages = {
        "OVERVIEW": overview_page(),
        "ESC": root_page("ESC", ["OVERVIEW", "MODE", "STEERING", "DRIVE", "POWER", "LINK", "MANUAL TEST"]),
        "PERCEPTION": root_page("PERCEPTION", ["OVERVIEW", "CAMERA", "DETECTION", "LANE", "OBSTACLE", "PERFORMANCE", "TEST"]),
        "NAVIGATION": root_page("NAVIGATION", ["OVERVIEW", "LOCALIZATION", "MISSION", "NAV2", "SAFETY", "TEST"]),
        "SYSTEM": root_page("SYSTEM", ["HEALTH", "IO PINS", "DISPLAY PINS", "LINKS", "ERRORS"]),
        "MANUAL_TEST": manual_page(),
    }
    for name, image in pages.items():
        assert image.size == (W, H)
        image.save(OUT / f"{name}.png")
    sheet = Image.new("RGB", (W * 3 + 40, H * 2 + 55), (20, 27, 34))
    draw = ImageDraw.Draw(sheet)
    for index, (name, image) in enumerate(pages.items()):
        row, col = divmod(index, 3)
        x, y = 10 + col * (W + 10), 30 + row * (H + 20)
        label(draw, (x, y - 16), name, F10, TEXT)
        sheet.paste(image, (x, y))
    sheet.save(OUT / "CURRENT_HMI_CONTACT_SHEET.png")
    print("PASS CURRENT_HMI_VISUAL_RENDER")
    for path in sorted(OUT.glob("*.png")):
        print(path.name, Image.open(path).size)

if __name__ == "__main__":
    main()
