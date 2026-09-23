#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include "TelemetryProtocol.h"
uint32_t crc(const std::string&s){uint32_t c=0xffffffffU;for(unsigned char x:s){c^=x;for(int i=0;i<8;i++)c=(c&1U)?((c>>1)^0xedb88320U):(c>>1);}return c^0xffffffffU;}
std::string wire(const char*d,const char*g,unsigned seq,const std::string&p){
 char b[420]; std::snprintf(b,sizeof(b),"F4X3:%s:%s:3:42:%u:10:%u:%s",d,g,seq,(unsigned)p.size(),p.c_str());
 std::string body=b; char out[460]; std::snprintf(out,sizeof(out),"%s:%08X",body.c_str(),crc(body)); return out;
}
int main(){
 VehicleTelemetry t=defaultTelemetry();
 auto cam=parseExtendedTelemetryLine(wire("PER","CAM",1,"1,1,12.5,8.2,0,0,ROS,1").c_str(),t,1000,42);
 assert(cam.accepted&&t.cameraReady&&t.perceptionInference);
 auto imu=parseExtendedTelemetryLine(wire("NAV","IMU",1,"1,12.5,12.4,0.1").c_str(),t,1010,42);
 assert(imu.accepted&&t.navx.imuYawDeg>12.4f);
 auto nav=parseExtendedTelemetryLine(wire("NAV","NAV2",1,"1,1,1,25,0.2,0.1,READY,READY,WAIT").c_str(),t,1020,42);
 assert(nav.accepted&&t.navx.nav2StackReady&&t.navx.pathValid);
 auto dup=parseExtendedTelemetryLine(wire("NAV","NAV2",1,"1,1,1,25,0.2,0.1,READY,READY,WAIT").c_str(),t,1030,42);
 assert(dup.recognized&&!dup.accepted&&dup.outOfOrder);
 auto wrong=parseExtendedTelemetryLine(wire("PER","CAM",2,"1,1,12.5,8.2,0,0,ROS,1").c_str(),t,1040,77);
 assert(wrong.recognized&&!wrong.accepted&&wrong.sessionError);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
 src=Path(td)/'check.cpp'; exe=Path(td)/'check'; src.write_text(CPP)
 cmd=['g++','-std=c++17','-Wall','-Wextra','-Werror','-DBOARD_F103_FAMILY','-DBOARD_F103C8','-I',str(ROOT/'src'),str(src),str(ROOT/'src/TelemetryProtocolF103.cpp'),str(ROOT/'src/NumericParse.cpp'),'-o',str(exe)]
 subprocess.run(cmd,check=True); subprocess.run([str(exe)],check=True)
print('V3_F103C8_TELEMETRY_SELF_CHECK_PASS')
