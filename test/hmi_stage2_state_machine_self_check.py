#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
cpp=r'''
#include <cassert>
#include <set>
#include <iostream>
#include "UiMenu.h"
int main(){
  struct D{UiMenuId root;int cards;int pages;};
  D ds[]={{UiMenuId::ESC_ROOT,12,4},{UiMenuId::PERCEPTION_ROOT,9,3},{UiMenuId::NAVIGATION_ROOT,12,4},{UiMenuId::SYSTEM_ROOT,10,4}};
  std::set<int> operator_ids;
  for(const auto &d:ds){
    uint8_t count=0; const UiMenuId *c=menuChildren(d.root,count);
    assert(c && count==d.cards && menuPageCount(count)==d.pages);
    for(int i=0;i<500;i++){
      const uint8_t page=static_cast<uint8_t>(i%d.pages);
      for(uint8_t slot=0;slot<3;slot++){
        const UiMenuId id=menuCardAt(d.root,page,slot);
        const int absolute=page*3+slot;
        if(absolute<d.cards) assert(id!=UiMenuId::SPLASH); else assert(id==UiMenuId::SPLASH);
      }
    }
    if(d.root!=UiMenuId::SYSTEM_ROOT) for(int i=0;i<d.cards;i++) assert(operator_ids.insert((int)c[i]).second);
  }
  assert(operator_ids.size()==33);
  uint8_t main_count=0; const UiMenuId *m=menuChildren(UiMenuId::MAIN_MENU,main_count);
  assert(m && main_count==3 && m[0]==UiMenuId::ESC_ROOT && m[1]==UiMenuId::PERCEPTION_ROOT && m[2]==UiMenuId::NAVIGATION_ROOT);
  assert(menuParent(UiMenuId::ESC_ROOT)==UiMenuId::MAIN_MENU);
  assert(menuParent(UiMenuId::PERCEPTION_ROOT)==UiMenuId::MAIN_MENU);
  assert(menuParent(UiMenuId::NAVIGATION_ROOT)==UiMenuId::MAIN_MENU);
  assert(menuParent(UiMenuId::SYSTEM_ROOT)==UiMenuId::HOME);
  std::cout<<"PASS HMI_STAGE2_STATE_MACHINE_500_TRANSITIONS\n";
}
'''
with tempfile.TemporaryDirectory() as td:
    src=Path(td)/'check.cpp'; exe=Path(td)/'check'
    src.write_text(cpp)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'src'),str(src),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS 33 operator cards unique + 500 page transitions/domain')