#pragma once
#include "Bts7960Winch.h"
#include "HmiConfig.h"
#include "HmiDisplay.h"
#include "Telemetry.h"
#include <cstdio>

extern HmiDisplay tft;

class HmiOperatorUi {
public:
  enum class Page : uint8_t { HOME=0, MAIN_MENU, ESC, PERCEPTION, NAVIGATION, FORK };

  void reset() {
    touchDown_=false;
    pressed_=K_NONE;
    emergencyStopRequested_=false;
    selectedFiveSeconds_=false;
    activeAction_=Action::NONE;
    actionRejected_=false;
    lastState_=Bts7960Winch::state();
    lastBusy_=actuatorBusy();
  }

  void cancelTouch() { touchDown_=false; pressed_=K_NONE; }
  bool touchDown() const { return touchDown_; }
  bool consumeEmergencyStopRequest() {
    const bool requested=emergencyStopRequested_;
    emergencyStopRequested_=false;
    return requested;
  }
  void showPage(Page) { cancelTouch(); }
  const char *wireName() const { return "HOME"; }

  bool serviceActuatorState() {
    const Bts7960WinchState state=Bts7960Winch::state();
    const bool busy=actuatorBusy();
    bool changed=state!=lastState_ || busy!=lastBusy_;
    if(!busy && activeAction_!=Action::NONE) {
      activeAction_=Action::NONE;
      changed=true;
    }
    lastState_=state;
    lastBusy_=busy;
    return changed;
  }

  void draw(const VehicleTelemetry &, bool fullRedraw=true) {
    if(fullRedraw) {
      tft.fillScreen(C_BG);
      drawStatic();
    }
    drawHeaderStatus();
    drawActions();
    drawDuration();
    drawBottomStatus();
  }

  bool pollTouch(const VehicleTelemetry &) {
    uint16_t x=0U,y=0U;
    const bool down=tft.getTouch(&x,&y,TOUCH_THRESHOLD);

    if(down && !touchDown_) {
      touchDown_=true;
      pressed_=hitKey(static_cast<int>(x),static_cast<int>(y));
      if(pressed_==K_ESTOP) {
        emergencyStopRequested_=true;
        return true;
      }
      if(pressed_==K_TIME_1) {
        if(!actuatorBusy()) {
          selectedFiveSeconds_=false;
          actionRejected_=false;
        }
        return true;
      }
      if(pressed_==K_TIME_5) {
        if(!actuatorBusy()) {
          selectedFiveSeconds_=true;
          actionRejected_=false;
        }
        return true;
      }
      if(isActionKey(pressed_)) {
        startAction(pressed_);
        return true;
      }
      return false;
    }

    if(!down && touchDown_) {
      touchDown_=false;
      const bool hadKey=pressed_!=K_NONE;
      pressed_=K_NONE;
      return hadKey;
    }
    return false;
  }

private:
  enum Key:int {
    K_NONE=0,
    K_FORK_UP,
    K_FORK_DOWN,
    K_WINCH_UP,
    K_TIME_1,
    K_TIME_5,
    K_ESTOP
  };
  enum class Action:uint8_t { NONE=0, FORK_UP, FORK_DOWN, WINCH_UP };

  static constexpr uint16_t C_BG=RGB565(6,13,22);
  static constexpr uint16_t C_PANEL=RGB565(16,30,44);
  static constexpr uint16_t C_PANEL_ACTIVE=RGB565(12,70,88);
  static constexpr uint16_t C_PANEL_DISABLED=RGB565(25,34,43);
  static constexpr uint16_t C_TEXT=RGB565(245,248,250);
  static constexpr uint16_t C_MUTED=RGB565(143,163,181);
  static constexpr uint16_t C_ACCENT=RGB565(0,190,230);
  static constexpr uint16_t C_GREEN=RGB565(58,210,112);
  static constexpr uint16_t C_RED=RGB565(238,78,78);
  static constexpr uint16_t C_BORDER=RGB565(55,81,105);

  bool touchDown_{false};
  int pressed_{K_NONE};
  bool emergencyStopRequested_{false};
  bool selectedFiveSeconds_{false};
  Action activeAction_{Action::NONE};
  bool actionRejected_{false};
  Bts7960WinchState lastState_{Bts7960WinchState::STOPPED};
  bool lastBusy_{false};

  static bool hit(int px,int py,int x,int y,int w,int h) {
    return px>=x && px<x+w && py>=y && py<y+h;
  }

  static bool isActionKey(int key) {
    return key==K_FORK_UP || key==K_FORK_DOWN || key==K_WINCH_UP;
  }

  bool actuatorBusy() const {
    return Bts7960Winch::direction()!=0 || Bts7960Winch::motionPending();
  }

  uint8_t durationSeconds() const { return selectedFiveSeconds_ ? 5U : 1U; }

  void text(const char *s,int x,int y,uint16_t fg,uint16_t bg,
            uint8_t datum=TL_DATUM,uint8_t size=1) {
    tft.setFreeFont(nullptr);
    tft.setTextFont(1);
    tft.setTextSize(size);
    tft.setTextDatum(datum);
    tft.setTextColor(fg,bg);
    tft.drawString(s?s:"-",x,y);
  }

  void frameRect(int x,int y,int w,int h,uint16_t color) {
    tft.drawFastHLine(x,y,w,color);
    tft.drawFastHLine(x,y+h-1,w,color);
    tft.drawFastVLine(x,y,h,color);
    tft.drawFastVLine(x+w-1,y,h,color);
  }

  void drawArrow(int cx,int cy,bool up,uint16_t color) {
    if(up) {
      tft.fillTriangle(cx,cy-8,cx-7,cy,cx+7,cy,color);
      tft.fillRect(cx-2,cy,5,9,color);
    } else {
      tft.fillRect(cx-2,cy-8,5,9,color);
      tft.fillTriangle(cx,cy+8,cx-7,cy,cx+7,cy,color);
    }
  }

  bool actionMatches(int key) const {
    return (key==K_FORK_UP && activeAction_==Action::FORK_UP) ||
           (key==K_FORK_DOWN && activeAction_==Action::FORK_DOWN) ||
           (key==K_WINCH_UP && activeAction_==Action::WINCH_UP);
  }

  void actionButton(int key,int x,int y,int w,int h,const char *label,bool up) {
    const bool busy=actuatorBusy();
    const bool active=busy && actionMatches(key);
    const bool disabled=busy && !active;
    const bool pressed=pressed_==key;
    const uint16_t fill=pressed ? C_PANEL_ACTIVE :
                        active ? C_PANEL_ACTIVE :
                        disabled ? C_PANEL_DISABLED : C_PANEL;
    const uint16_t border=active ? C_GREEN :
                          pressed ? C_TEXT :
                          disabled ? C_BORDER : C_ACCENT;
    const uint16_t fg=disabled ? C_MUTED : C_TEXT;
    tft.fillRect(x,y,w,h,fill);
    frameRect(x,y,w,h,border);
    drawArrow(x+20,y+h/2,up,fg);
    text(label,x+w/2+8,y+h/2-7,fg,fill,MC_DATUM,2);
  }

  void durationButton(int key,int x,int y,int w,const char *label,bool selected) {
    const bool busy=actuatorBusy();
    const bool pressed=pressed_==key && !busy;
    const uint16_t fill=busy ? C_PANEL_DISABLED :
                        (selected || pressed) ? C_PANEL_ACTIVE : C_PANEL;
    const uint16_t border=busy ? C_BORDER :
                          selected ? C_ACCENT : C_BORDER;
    const uint16_t fg=busy ? C_MUTED : C_TEXT;
    tft.fillRect(x,y,w,34,fill);
    frameRect(x,y,w,34,border);
    if(selected) tft.fillRect(x+8,y+13,7,7,busy ? C_MUTED : C_GREEN);
    text(label,x+w/2+5,y+10,fg,fill,MC_DATUM,1);
  }

  void drawStatic() {
    text("FORKLIFT CONTROL",8,9,C_TEXT,C_BG,TL_DATUM,1);
    tft.drawFastHLine(8,31,304,C_BORDER);
    text("DURATION",8,158,C_MUTED,C_BG,TL_DATUM,1);
  }

  void drawHeaderStatus() {
    const bool fault=Bts7960Winch::faulted();
    const bool busy=actuatorBusy();
    const char *label=fault ? "FAULT" : (busy ? "BUSY" : "READY");
    const uint16_t color=fault ? C_RED : (busy ? C_ACCENT : C_GREEN);
    tft.fillRect(180,2,132,27,C_BG);
    text(label,244,9,color,C_BG,TR_DATUM,1);
    const uint16_t stopFill=pressed_==K_ESTOP ? C_RED : C_PANEL;
    tft.fillRect(252,3,60,24,stopFill);
    frameRect(252,3,60,24,C_RED);
    text("STOP",282,9,C_TEXT,stopFill,MC_DATUM,1);
  }

  void drawActions() {
    actionButton(K_FORK_UP,8,40,148,55,"FORK UP",true);
    actionButton(K_FORK_DOWN,164,40,148,55,"FORK DOWN",false);
    actionButton(K_WINCH_UP,8,102,304,48,"WINCH UP",true);
  }

  void drawDuration() {
    durationButton(K_TIME_1,8,174,148,"1 SECOND",!selectedFiveSeconds_);
    durationButton(K_TIME_5,164,174,148,"5 SECOND",selectedFiveSeconds_);
  }

  const char *activeLabel() const {
    switch(activeAction_) {
      case Action::FORK_UP: return "FORK UP";
      case Action::FORK_DOWN: return "FORK DOWN";
      case Action::WINCH_UP: return "WINCH UP";
      default: return "READY";
    }
  }

  void drawBottomStatus() {
    tft.fillRect(0,214,320,26,C_PANEL);
    char line[32];
    uint16_t color=C_GREEN;
    if(Bts7960Winch::faulted()) {
      std::snprintf(line,sizeof(line),"STATUS: FAULT");
      color=C_RED;
    } else if(Bts7960Winch::state()==Bts7960WinchState::TOP_LIMIT) {
      std::snprintf(line,sizeof(line),"STATUS: TOP LIMIT");
      color=C_RED;
    } else if(Bts7960Winch::state()==Bts7960WinchState::BOTTOM_LIMIT) {
      std::snprintf(line,sizeof(line),"STATUS: BOTTOM LIMIT");
      color=C_RED;
    } else if(actuatorBusy() && activeAction_!=Action::NONE) {
      std::snprintf(line,sizeof(line),"%s %us PWM:%u",
                    activeLabel(), static_cast<unsigned>(durationSeconds()),
                    static_cast<unsigned>(Bts7960Winch::appliedPwm()));
      color=C_ACCENT;
    } else if(actionRejected_) {
      std::snprintf(line,sizeof(line),"INTERLOCK T:%u B:%u",
                    Bts7960Winch::topLimitActive()?1U:0U,
                    Bts7960Winch::bottomLimitActive()?1U:0U);
      color=C_RED;
    } else {
      std::snprintf(line,sizeof(line),"READY PWM:%u T:%u B:%u",
                    static_cast<unsigned>(Bts7960Winch::configuredPwm()),
                    Bts7960Winch::topLimitActive()?1U:0U,
                    Bts7960Winch::bottomLimitActive()?1U:0U);
    }
    text(line,8,222,color,C_PANEL,ML_DATUM,1);
  }

  void startAction(int key) {
    if(actuatorBusy()) return;
    actionRejected_=false;

    // A brief electrical transient can leave the contradictory-limit fault
    // latched even after the inputs have returned to a valid single-end-stop
    // state. Recover that stale local actuator fault before a new operator
    // request. clearFault() itself still enforces idle, E-stop, vehicle-safe,
    // and non-contradictory limit conditions.
    if(Bts7960Winch::faulted())
      (void)Bts7960Winch::clearFault();

    // A previously persisted PWM=0 is an actuator-disable value. For a direct
    // local operator request, restore the existing firmware default in RAM only
    // so a valid touch cannot silently produce no motion. Do not rewrite flash
    // or alter the configured tuning unless the operator explicitly saves it.
    if(Bts7960Winch::configuredPwm()==0U)
      (void)Bts7960Winch::resetPwm(false);
    if(key==K_FORK_DOWN) {
      if(selectedFiveSeconds_) Bts7960Winch::downTimed2Local();
      else Bts7960Winch::downTimed1Local();
      if(actuatorBusy()) activeAction_=Action::FORK_DOWN;
    } else {
      if(selectedFiveSeconds_) Bts7960Winch::upTimed2Local();
      else Bts7960Winch::upTimed1Local();
      if(actuatorBusy())
        activeAction_=key==K_WINCH_UP ? Action::WINCH_UP : Action::FORK_UP;
    }
    if(!actuatorBusy()) actionRejected_=true;
    lastState_=Bts7960Winch::state();
    lastBusy_=actuatorBusy();
  }

  int hitKey(int x,int y) const {
    if(hit(x,y,248,0,72,32)) return K_ESTOP;
    if(hit(x,y,8,40,148,55)) return K_FORK_UP;
    if(hit(x,y,164,40,148,55)) return K_FORK_DOWN;
    if(hit(x,y,8,102,304,48)) return K_WINCH_UP;
    if(hit(x,y,8,174,148,34)) return K_TIME_1;
    if(hit(x,y,164,174,148,34)) return K_TIME_5;
    return K_NONE;
  }
};
