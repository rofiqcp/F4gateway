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

def carousel(draw, selected, count, actionable=False):
    y = 188
    boxes = [(4, y, 105, 235), (110, y, 211, 235), (216, y, 317, 235)]
    for i, box in enumerate(boxes):
        round_box(draw, box, PANEL if i != 1 or actionable else PANEL2,
                  ACCENT if i == 1 and actionable else BORDER)
    label(draw, (54, 212), "<  LEFT", F10, TEXT, "mm")
    label(draw, (160, 212), "OK" if actionable else f"{selected + 1} / {count}",
          F12, ACCENT if actionable else DIM, "mm")
    label(draw, (267, 212), "RIGHT  >", F10, TEXT, "mm")

def root_page(title, items, selected=0):
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    topbar(draw, title)
    for slot, item in enumerate(items[:3]):
        x = 6 + slot * 104
        round_box(draw, (x, 56, x + 99, 183), PANEL2 if slot == selected else PANEL,
                  ACCENT if slot == selected else BORDER)
        label(draw, (x + 50, 92), item, F10, ACCENT if slot == selected else TEXT, "mm")
        label(draw, (x + 50, 126), "READY", F8, READY, "mm")
        label(draw, (x + 50, 162), str(slot + 1), F8, DIM, "mm")
    carousel(draw, selected, len(items))
    return image
def overview_page():
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    topbar(draw, "OVERVIEW")
    for i, item in enumerate(("ESC", "PERCEPTION", "NAVIGATION")):
        x = 6 + i * 104
        round_box(draw, (x, 56, x + 99, 183), PANEL, BORDER)
        label(draw, (x + 50, 98), item, F10, TEXT, "mm")
        label(draw, (x + 50, 146), "READY", F8, READY, "mm")
    label(draw, (160, 38), "ESC / PERCEPTION / NAV2", F8, DIM, "mm")
    return image

def manual_page():
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    topbar(draw, "MANUAL TEST")
    middle = [(6, 56, 105, 183), (110, 56, 209, 183), (214, 56, 313, 183)]
    for box, text, color in zip(middle, ("LEFT", "STOP", "RIGHT"), (ACCENT, FAULT, ACCENT)):
        round_box(draw, box, FAULT if text == "STOP" else PANEL, color)
        label(draw, ((box[0]+box[2])//2, 120), text, F10, TEXT, "mm")
    bottom = [(4,188,105,235),(110,188,211,235),(216,188,317,235)]
    for box, text, color in zip(bottom, ("REV", "STOP", "FWD"), (ACCENT, FAULT, ACCENT)):
        round_box(draw, box, FAULT if text == "STOP" else PANEL, color)
        label(draw, ((box[0]+box[2])//2, 212), text, F10, TEXT, "mm")
    label(draw, (160, 38), "HOLD 0.6s | RELEASE = STOP", F8, (242,181,56), "mm")
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
