#pragma once
#include "BtsWinch.h"
#include "Config.h"
#include "HmiDisplay.h"
#include "Telemetry.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

extern HmiDisplay tft;

class F4V2Ui {
public:
  enum class Page : uint8_t { HOME=0, MAIN_MENU, ESC, PERCEPTION, NAVIGATION, FORK };

  void reset() { page_=Page::HOME; escSlide_=perSlide_=navSlide_=forkSlide_=0U; touchDown_=false; }
  const char *wireName() const {
    switch (page_) { case Page::HOME:return "HOME"; case Page::MAIN_MENU:return "MAIN_MENU";
      case Page::ESC:return "ESC"; case Page::PERCEPTION:return "PERCEPTION";
      case Page::NAVIGATION:return "NAVIGATION"; case Page::FORK:return "FORK"; }
    return "HOME";
  }

  void draw(const VehicleTelemetry &d) {
    tft.fillScreen(C_BG2); drawTop(d);
    switch(page_) { case Page::HOME: drawHome(d); break; case Page::MAIN_MENU: drawMain(d); break;
      case Page::ESC: drawEsc(d); break; case Page::PERCEPTION: drawPer(d); break;
      case Page::NAVIGATION: drawNav(d); break; case Page::FORK: drawFork(); break; }
  }

  bool pollTouch(const VehicleTelemetry &) {
    uint16_t x=0U,y=0U; const bool down=tft.getTouch(&x,&y,200U);
    if (down && !touchDown_) {
      touchDown_=true; pressed_=hitKey((int)x,(int)y);
      if (pressed_==K_STOP) { BtsWinch::stop(); return true; }
      return false;
    }
    if (!down && touchDown_) {
      touchDown_=false; const int key=pressed_; pressed_=K_NONE;
      return executeKey(key);
    }
    return false;
  }

private:
  static constexpr uint16_t C_BG2=RGB565(9,12,16), C_PANEL=RGB565(16,21,28), C_CARD=RGB565(23,30,39);
  static constexpr uint16_t C_BTN=RGB565(30,39,50), C_BORDER2=RGB565(56,68,82), C_TEXT2=RGB565(242,246,250);
  static constexpr uint16_t C_MUTED=RGB565(158,170,183), C_DIM=RGB565(96,108,121), C_ACC=RGB565(0,168,255);
  static constexpr uint16_t C_GREEN2=RGB565(45,204,112), C_WARN=RGB565(245,166,35), C_RED2=RGB565(235,79,79), C_WHITE2=0xFFFFU;
  static constexpr int W2=320,H2=240,BAR_H=28;
  static constexpr int CARD_Y=38,CARD_H=136,CARD_W=98,CARD_GAP=5,X1=6,X2=109,X3=212;
  static constexpr int NAV_Y=183,NAV_H=49;
  enum Key:int { K_NONE=0,K_HOME,K_BACK,K_MENU,K_ESC,K_PER,K_NAV,K_FORK,K_LEFT,K_RIGHT,K_UP,K_STOP,K_DOWN };
  Page page_{Page::HOME}; uint8_t escSlide_{0},perSlide_{0},navSlide_{0},forkSlide_{0};
  bool touchDown_{false}; int pressed_{K_NONE};

  static bool hit(int px,int py,int x,int y,int w,int h){return px>=x&&px<x+w&&py>=y&&py<y+h;}
  static const char *yn(bool v){return v?"READY":"WAIT";}
  void text(const char*s,int x,int y,uint16_t c,uint16_t bg,uint8_t datum=TL_DATUM,uint8_t size=1){
    tft.setFreeFont(nullptr); tft.setTextFont(1); tft.setTextSize(size); tft.setTextDatum(datum); tft.setTextColor(c,bg); tft.drawString(s?s:"-",x,y);
  }
  void panel(int x,int y,int w,int h,uint16_t fill,uint16_t border){tft.fillRoundRect(x,y,w,h,8,fill);tft.drawRoundRect(x,y,w,h,8,border);}
  void dot(int x,int y,uint16_t c){tft.fillCircle(x,y,3,c);}
  void topButton(int x,const char *label){panel(x,2,58,24,C_PANEL,C_BORDER2);text(label,x+29,9,C_TEXT2,C_PANEL,TC_DATUM,1);}
  void drawTop(const VehicleTelemetry &d){
    tft.fillRect(0,0,W2,BAR_H,C_BG2);
    const char *title=wireName(); text(title,W2/2,8,C_TEXT2,C_BG2,TC_DATUM,1);
    if(page_!=Page::HOME){topButton(4,page_==Page::MAIN_MENU?"HOME":"BACK");}
    dot(273,14,d.rosConnected?C_GREEN2:C_DIM); dot(287,14,d.escFresh?C_GREEN2:C_DIM); dot(301,14,d.eStop?C_RED2:C_GREEN2);
  }
  void card(int x,const char*title,const char*value,const char*foot,uint16_t accent){
    panel(x,CARD_Y,CARD_W,CARD_H,C_CARD,accent);dot(x+12,CARD_Y+14,accent);
    text(title,x+CARD_W/2,CARD_Y+16,C_MUTED,C_CARD,TC_DATUM,1);
    text(value,x+CARD_W/2,CARD_Y+68,C_TEXT2,C_CARD,MC_DATUM,2);
    text(foot,x+CARD_W/2,CARD_Y+CARD_H-10,C_MUTED,C_CARD,BC_DATUM,1);
  }
  void drawHome(const VehicleTelemetry &d){
    char speed[20]; std::snprintf(speed,sizeof(speed),"%.1f",d.speedKmh);
    panel(6,34,98,138,C_CARD,C_ACC); dot(18,48,d.nav2Ready?C_GREEN2:C_DIM); text("NAV",55,50,C_MUTED,C_CARD,TC_DATUM); text(d.navigationStatus==NAV_NAVIGATING?"ACTIVE":"IDLE",55,104,C_TEXT2,C_CARD,MC_DATUM,2); text(d.activeTarget,55,158,C_MUTED,C_CARD,BC_DATUM);
    panel(109,34,98,138,C_CARD,C_ACC); dot(121,48,d.escFresh?C_GREEN2:C_DIM); text("SPEED",158,50,C_MUTED,C_CARD,TC_DATUM); text(speed,158,99,C_TEXT2,C_CARD,MC_DATUM,2); text("km/h",158,124,C_MUTED,C_CARD,MC_DATUM); text(d.state==STATE_RUNNING?"MOVING":"STOP",158,158,C_MUTED,C_CARD,BC_DATUM);
    panel(212,34,98,138,C_CARD,d.perceptionReady?C_GREEN2:C_BORDER2); dot(224,48,d.perceptionReady?C_GREEN2:C_DIM); text("PERCEPTION",261,50,C_MUTED,C_CARD,TC_DATUM); text(d.perceptionReady?"ONLINE":"OFF",261,104,C_TEXT2,C_CARD,MC_DATUM,2); text(d.detectedObject,261,158,C_MUTED,C_CARD,BC_DATUM);
    panel(74,181,172,51,C_BTN,C_ACC); text("MENU",160,198,C_ACC,C_BTN,MC_DATUM,2);
  }
  void menuCard(int x,int y,const char*a,const char*b,uint16_t c){panel(x,y,150,76,C_CARD,c);dot(x+13,y+13,c);text(a,x+75,y+30,C_TEXT2,C_CARD,MC_DATUM,b&&*b?1:2);if(b&&*b)text(b,x+75,y+48,C_MUTED,C_CARD,MC_DATUM);}
  void drawMain(const VehicleTelemetry &){menuCard(6,38,"ESC / SPEED","DRIVE",C_ACC);menuCard(164,38,"PERCEPTION","CAMERA",C_GREEN2);menuCard(6,122,"NAVIGATION","NAV2",C_ACC);menuCard(164,122,"FORK","BTS7960",C_WARN);}
  void footer(uint8_t slide){panel(6,NAV_Y,91,NAV_H,C_BTN,C_BORDER2);text("< LEFT",51,NAV_Y+18,C_TEXT2,C_BTN,MC_DATUM);panel(104,NAV_Y,112,NAV_H,C_PANEL,C_BORDER2);char p[12];std::snprintf(p,sizeof(p),"%u / 3",(unsigned)slide+1U);text(p,160,NAV_Y+18,C_ACC,C_PANEL,MC_DATUM);panel(223,NAV_Y,91,NAV_H,C_BTN,C_BORDER2);text("RIGHT >",268,NAV_Y+18,C_TEXT2,C_BTN,MC_DATUM);}
  void drawEsc(const VehicleTelemetry &d){char a[22],b[22],c[22];const uint8_t s=escSlide_; if(s==0){std::snprintf(a,sizeof(a),"%.0f",d.escx.leftRpm);std::snprintf(b,sizeof(b),"%.0f",d.escx.rightRpm);std::snprintf(c,sizeof(c),"%.1f",d.steeringActualDeg);card(X1,"LEFT MOTOR",a,"rpm",d.escReady?C_GREEN2:C_WARN);card(X2,"RIGHT MOTOR",b,"rpm",d.escReady?C_GREEN2:C_WARN);card(X3,"STEERING",c,"deg",d.encoderReady?C_ACC:C_WARN);}else if(s==1){card(X1,"ESC OVERVIEW",d.escFresh?"ONLINE":"OFF","MOTOR LINK",d.escFresh?C_GREEN2:C_RED2);card(X2,"OPERATOR",d.state==STATE_RUNNING?"RUN":"STOP","MOTION",d.state==STATE_RUNNING?C_GREEN2:C_DIM);card(X3,"STEERING",yn(d.encoderReady),"POSITION",d.encoderReady?C_GREEN2:C_WARN);}else{card(X1,"LEFT STATUS",yn(d.escReady),"ESC LEFT",d.escReady?C_GREEN2:C_WARN);card(X2,"RIGHT STATUS",yn(d.vescConnected),"ESC RIGHT",d.vescConnected?C_GREEN2:C_WARN);card(X3,"SYSTEM",d.escFresh?"HEALTHY":"CHECK","ESC HEALTH",d.escFresh?C_GREEN2:C_RED2);}footer(s);}
  void drawPer(const VehicleTelemetry &d){char fps[20];std::snprintf(fps,sizeof(fps),"%.0f fps",d.cameraFps);const uint8_t s=perSlide_;if(s==0){card(X1,"PER OVERVIEW",d.perceptionReady?"ONLINE":"OFF","PERCEPTION",d.perceptionReady?C_GREEN2:C_RED2);card(X2,"CAMERA",d.cameraReady?"ON":"OFF","IMAGE SOURCE",d.cameraReady?C_GREEN2:C_DIM);card(X3,"DETECTION",d.perceptionInference?"ON":"OFF","YOLO / OBJECT",d.perceptionInference?C_ACC:C_DIM);}else if(s==1){card(X1,"CAMERA LINK",d.cameraReady?"OK":"LOST","USB CAMERA",d.cameraReady?C_GREEN2:C_RED2);card(X2,"FRAME RATE",fps,"STREAM",C_ACC);card(X3,"CAM STATUS",d.cameraReady?"ACTIVE":"IDLE","CAPTURE",d.cameraReady?C_GREEN2:C_DIM);}else{card(X1,"DETECTION",d.perceptionInference?"ACTIVE":"OFF","OBJECT MODEL",d.perceptionInference?C_ACC:C_DIM);card(X2,"OBSTACLE",d.obstacleDetected?"DETECTED":"CLEAR","SAFETY",d.obstacleDetected?C_RED2:C_GREEN2);card(X3,"PER HEALTH",d.perceptionReady?"HEALTHY":"CHECK","SYSTEM",d.perceptionReady?C_GREEN2:C_RED2);}footer(s);}
  void drawNav(const VehicleTelemetry &d){char dist[20],head[20],spd[20];std::snprintf(dist,sizeof(dist),"%.1f m",d.objectDistanceM);std::snprintf(head,sizeof(head),"%.1f",d.headingDeg);std::snprintf(spd,sizeof(spd),"%.1f",d.driveActualMps);const uint8_t s=navSlide_;if(s==0){card(X1,"NAV OVERVIEW",d.navigationStatus==NAV_NAVIGATING?"ACTIVE":"IDLE",navigationStatusText(d.navigationStatus),d.nav2Ready?C_GREEN2:C_DIM);card(X2,"GOAL",d.activeTarget,"TARGET",C_ACC);card(X3,"DISTANCE",dist,"TO GOAL",C_ACC);}else if(s==1){card(X1,"HEADING",head,"deg",C_ACC);card(X2,"SPEED",spd,"m/s",d.state==STATE_RUNNING?C_GREEN2:C_DIM);card(X3,"NAV LINK",d.nav2Ready?"ONLINE":"OFF","ROS / NAV2",d.nav2Ready?C_GREEN2:C_RED2);}else{card(X1,"MISSION",d.navigationStatus==NAV_NAVIGATING?"RUNNING":"STOP","NAV2",d.navigationStatus==NAV_NAVIGATING?C_GREEN2:C_DIM);card(X2,"GOAL",d.activeTarget,"MISSION",C_ACC);card(X3,"STATUS",navigationStatusText(d.navigationStatus),d.motionReady?"SYSTEM READY":"CHECK LINK",d.motionReady?C_GREEN2:C_RED2);}footer(s);}
  void forkCard(int x,const char*title,const char*sub,uint16_t c){panel(x,CARD_Y,CARD_W,CARD_H,C_CARD,c);dot(x+12,CARD_Y+14,c);text(title,x+CARD_W/2,CARD_Y+58,c,C_CARD,MC_DATUM);text(sub,x+CARD_W/2,CARD_Y+82,C_TEXT2,C_CARD,MC_DATUM,2);text("READY",x+CARD_W/2,CARD_Y+CARD_H-10,C_MUTED,C_CARD,BC_DATUM);}
  void drawFork(){const uint8_t s=forkSlide_;forkCard(X1,"UP",s==0?"HOME":(s==1?"1 SEC":"2 SEC"),C_GREEN2);forkCard(X2,"STOP","",C_RED2);forkCard(X3,"DOWN",s==0?"HOME":(s==1?"1 SEC":"2 SEC"),C_ACC);footer(s);char st[36];std::snprintf(st,sizeof(st),"%s PWM:%u T:%u B:%u",BtsWinch::stateName(),(unsigned)BtsWinch::appliedPwm(),BtsWinch::topLimitActive()?1U:0U,BtsWinch::bottomLimitActive()?1U:0U);text(st,160,29,BtsWinch::faulted()?C_RED2:C_MUTED,C_BG2,TC_DATUM);}

  uint8_t &slide(){if(page_==Page::ESC)return escSlide_;if(page_==Page::PERCEPTION)return perSlide_;if(page_==Page::NAVIGATION)return navSlide_;return forkSlide_;}
  int hitKey(int x,int y){
    if(page_!=Page::HOME && hit(x,y,0,0,96,30)) return page_==Page::MAIN_MENU?K_HOME:K_BACK;
    if(page_==Page::HOME) return hit(x,y,74,181,172,51)?K_MENU:K_NONE;
    if(page_==Page::MAIN_MENU){if(hit(x,y,6,38,150,76))return K_ESC;if(hit(x,y,164,38,150,76))return K_PER;if(hit(x,y,6,122,150,76))return K_NAV;if(hit(x,y,164,122,150,76))return K_FORK;return K_NONE;}
    if(hit(x,y,6,NAV_Y,91,NAV_H))return K_LEFT;if(hit(x,y,223,NAV_Y,91,NAV_H))return K_RIGHT;
    if(page_==Page::FORK&&hit(x,y,X1,CARD_Y,CARD_W,CARD_H))return K_UP;
    if(page_==Page::FORK&&hit(x,y,X2,CARD_Y,CARD_W,CARD_H))return K_STOP;
    if(page_==Page::FORK&&hit(x,y,X3,CARD_Y,CARD_W,CARD_H))return K_DOWN;
    return K_NONE;
  }
  bool executeKey(int k){
    if(k==K_NONE||k==K_STOP)return k==K_STOP;
    if(k==K_HOME){page_=Page::HOME;return true;} if(k==K_BACK){page_=Page::MAIN_MENU;return true;} if(k==K_MENU){page_=Page::MAIN_MENU;return true;}
    if(k==K_ESC){page_=Page::ESC;return true;}if(k==K_PER){page_=Page::PERCEPTION;return true;}if(k==K_NAV){page_=Page::NAVIGATION;return true;}if(k==K_FORK){page_=Page::FORK;return true;}
    if(k==K_LEFT){uint8_t&s=slide();s=s==0?2:static_cast<uint8_t>(s-1);return true;}if(k==K_RIGHT){uint8_t&s=slide();s=static_cast<uint8_t>((s+1)%3);return true;}
    if(page_==Page::FORK&&k==K_UP){if(forkSlide_==0)BtsWinch::upHome();else if(forkSlide_==1)BtsWinch::upTimed1();else BtsWinch::upTimed2();return true;}
    if(page_==Page::FORK&&k==K_DOWN){if(forkSlide_==0)BtsWinch::downHome();else if(forkSlide_==1)BtsWinch::downTimed1();else BtsWinch::downTimed2();return true;}
    return false;
  }
};
