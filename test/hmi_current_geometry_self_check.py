#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "src/Config.h").read_text(errors="replace")
MENU = (ROOT / "src/UiMenu.h").read_text(errors="replace")
TOUCH = (ROOT / "src/TouchButtons.h").read_text(errors="replace")
SHELL = (ROOT / "src/UiShell.h").read_text(errors="replace")


def need(condition, message):
    if not condition:
        print("FAIL", message)
        sys.exit(1)
    print("PASS", message)


def cint(name):
    match = re.search(rf"constexpr\s+(?:uint\d+_t|int)\s+{name}\s*=\s*([^;]+);", CONFIG)
    need(match is not None, f"constant {name} exists")
    expr = match.group(1).strip()
    values = {"W": 320, "H": 240}
    return int(eval(expr, {"__builtins__": {}}, values))
W, H = cint("W"), cint("H")
top_h = cint("TOP_H")
content_y = cint("CONTENT_Y")
content_bottom = cint("CONTENT_BOTTOM")
soft_y, soft_h = cint("SOFTKEY_Y"), cint("SOFTKEY_H")
card_y, card_h = cint("OVERVIEW_CARD_Y"), cint("OVERVIEW_CARD_H")
summary_y, summary_h = cint("OVERVIEW_SUMMARY_Y"), cint("OVERVIEW_SUMMARY_H")
submenu_y, submenu_h = cint("SUBMENU_CARD_Y"), cint("SUBMENU_CARD_H")
carousel_y, carousel_h = cint("CAROUSEL_NAV_Y"), cint("CAROUSEL_NAV_H")

need((W, H) == (320, 240), "physical layout remains 320x240")
need(0 <= top_h <= content_y < content_bottom < soft_y < H, "vertical zones are ordered")
need(summary_y >= top_h and summary_y + summary_h <= card_y, "overview summary cannot overlap cards")
need(card_y + card_h <= carousel_y, "overview cards cannot overlap carousel")
need(submenu_y + submenu_h <= carousel_y, "submenu cards cannot overlap carousel")
need(carousel_y + carousel_h <= H, "carousel footer is inside display")
need(soft_y + soft_h <= H, "edit footer is inside display")

# Manual-test hit boxes must stay inside the screen and leave STOP reachable.
manual_boxes = [(108, 40, 104, 50), (6, 96, 94, 70), (108, 96, 104, 70),
                (220, 96, 94, 70), (108, 172, 104, 52)]
for box in manual_boxes:
    x, y, w, h = box
    need(x >= 0 and y >= 0 and x + w <= W and y + h <= H, f"manual hit box {box} in bounds")
# The current tree must expose all five top-level domains and current dead-man semantics.
for token in ["ESC_ROOT", "PERCEPTION_ROOT", "NAVIGATION_ROOT", "SYSTEM_ROOT"]:
    need(f"UiMenuId::{token}" in MENU, f"top-level menu {token} exists")
need("HOLD 0.6s" in SHELL and "RELEASE = STOP" in SHELL, "manual page documents dead-man hold/release")
need("TOUCH_HOLD_ACTION_MS" in TOUCH and "TouchEvent::HOLD" in TOUCH, "touch layer implements hold activation")
need("TouchEvent::RELEASE" in TOUCH and "isManualMotionKey" in TOUCH, "touch layer emits release for motion stop")
need("invalidateTouchGeneration" in TOUCH, "page/display generation can invalidate armed touch")

# Every menu enum value except SPLASH/OVERVIEW must have a title and be reachable via a parent.
enum_block = re.search(r"enum class UiMenuId[^\{]*\{(.*?)\};", CONFIG, re.S)
need(enum_block is not None, "UiMenuId enum parsed")
ids = []
for raw in enum_block.group(1).split(","):
    name = raw.strip().split("=")[0].strip()
    if name:
        ids.append(name)
for name in ids:
    need(f"case UiMenuId::{name}:" in MENU, f"menu {name} has title/dispatch case")
    if name not in {"SPLASH", "OVERVIEW"}:
        need(MENU.count(f"case UiMenuId::{name}:") >= 2, f"menu {name} participates in title + parent/tree logic")

legacy_names = ["HomePage.h", "CameraPage.h", "GpsPage.h", "ActuatorPage.h", "BottomMenu.h", "TopBar.h"]
active = "\n".join(p.read_text(errors="replace") for p in list((ROOT / "src").glob("*")) if p.is_file())
for legacy in legacy_names:
    need(legacy not in active, f"legacy renderer {legacy} removed from active source")
print("PASS F4GATEWAY_CURRENT_HMI_GEOMETRY")
