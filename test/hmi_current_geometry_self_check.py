#!/usr/bin/env python3
from pathlib import Path
import re, sys
ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT/'src/Config.h').read_text(errors='replace')
MENU = (ROOT/'src/UiMenu.h').read_text(errors='replace')
TOUCH = (ROOT/'src/TouchButtons.h').read_text(errors='replace')
SHELL = (ROOT/'src/UiShell.h').read_text(errors='replace')
MAIN = (ROOT/'src/main.cpp').read_text(errors='replace')

def need(c,m):
    if not c:
        print('FAIL',m); sys.exit(1)
    print('PASS',m)

def cint(name):
    m=re.search(rf'constexpr\s+(?:uint\d+_t|int)\s+{name}\s*=\s*([^;]+);',CONFIG)
    need(m is not None,f'constant {name} exists')
    return int(eval(m.group(1).strip(),{'__builtins__':{}},{'W':320,'H':240}))

W,H=cint('W'),cint('H')
top_h=cint('TOP_H'); submenu_y=cint('SUBMENU_CARD_Y'); submenu_h=cint('SUBMENU_CARD_H')
footer_y=cint('CAROUSEL_NAV_Y'); footer_h=cint('CAROUSEL_NAV_H')
soft_y=cint('SOFTKEY_Y'); soft_h=cint('SOFTKEY_H')
home_y=cint('HOME_TILE_Y'); home_h=cint('HOME_TILE_H')
home_menu_y=cint('HOME_MENU_Y'); home_menu_h=cint('HOME_MENU_H')
need((W,H)==(320,240),'physical layout remains 320x240')
need(top_h <= home_y and home_y+home_h < home_menu_y,'HOME summary tiles do not overlap MENU')
need(home_menu_y+home_menu_h <= H,'HOME MENU is inside display')
need(submenu_y+submenu_h <= footer_y,'domain cards do not overlap pager footer')
need(footer_y+footer_h <= H and soft_y+soft_h <= H,'pager/edit footers are in bounds')

# Three 100px columns + gaps stay inside 320px.
x0=cint('UI_CARD_X0'); cw=cint('UI_CARD_W'); gap=cint('UI_CARD_GAP')
need(x0 + 2*(cw+gap) + cw <= W,'three-card grid fits physical width')
need('DOMAIN_PAGE_SIZE = 3' in CONFIG,'page size is exactly three cards')
need('case UiMenuId::MAIN_MENU: count = 3U; return mainMenu;' in MENU,'MAIN MENU has exactly three cards')
main_arr=re.search(r'static const UiMenuId mainMenu\[\]\s*=\s*\{([^}]*)\}',MENU,re.S)
need(main_arr is not None,'MAIN MENU descriptor exists')
body=main_arr.group(1)
for token in ['ESC_ROOT','PERCEPTION_ROOT','NAVIGATION_ROOT']:
    need(f'UiMenuId::{token}' in body,f'MAIN MENU contains {token}')
need('SYSTEM_ROOT' not in body,'SYSTEM is hidden from operator MAIN MENU')

for root,count,pages in [('ESC_ROOT',12,4),('PERCEPTION_ROOT',9,3),('NAVIGATION_ROOT',12,4)]:
    need(f'case UiMenuId::{root}: count = {count}U;' in MENU,f'{root} exposes {count} cards')
    need(f'UiMenuId::{root}' in MENU and 'menuPageCount' in MENU,f'{root} uses page-of-three helper')
need('pageIndex{0U}' in MENU and 'detailViewIndex{0U}' in MENU,'UI state stores page + detail view')
need('menuPageFirst' in MENU and 'menuCardAt' in MENU,'page/card mapping helpers exist')
need('selectedChild' not in MENU+MAIN+SHELL,'legacy sliding selectedChild removed')
need('menuWindowFirst' not in MENU+MAIN+SHELL,'legacy sliding window helper removed')

manual_boxes=[(108,40,104,50),(6,96,94,70),(108,96,104,70),(220,96,94,70),(108,172,104,52)]
for box in manual_boxes:
    x,y,w,h=box
    need(x>=0 and y>=0 and x+w<=W and y+h<=H,f'manual hit box {box} in bounds')
need('HOLD 0.6s' in SHELL and 'RELEASE = STOP' in SHELL,'manual page documents dead-man semantics')
need('SERVICE_HOLD_MS = 800' in CONFIG and 'suppressTouchUntilRelease' in TOUCH,'service long-hold is one-shot and release-suppressed')
need('UiMenuId::HOME' in MAIN and 'UiMenuId::MAIN_MENU' in MAIN,'runtime exposes HOME and MAIN MENU states')
need('gUi.menu = UiMenuId::HOME' in MAIN,'splash completes into HOME')

legacy=['HomePage.h','CameraPage.h','GpsPage.h','ActuatorPage.h','BottomMenu.h','TopBar.h']
active='\n'.join(p.read_text(errors='replace') for p in (ROOT/'src').glob('*') if p.is_file())
for name in legacy: need(name not in active,f'legacy renderer {name} absent')
print('PASS F4GATEWAY_STAGE1_HMI_GEOMETRY')
