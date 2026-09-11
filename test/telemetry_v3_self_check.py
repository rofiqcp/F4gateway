#!/usr/bin/env python3
from pathlib import Path
import subprocess, tempfile
ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include "TelemetryProtocol.h"
uint32_t crc32(const std::string &s){uint32_t c=0xffffffffU;for(unsigned char x:s){c^=x;for(int i=0;i<8;i++)c=(c&1U)?((c>>1)^0xedb88320U):(c>>1);}return c^0xffffffffU;}
std::string v3(const char*d,const char*g,uint32_t session,uint32_t seq,uint32_t age,const std::string&p,int schema=3,int len_adjust=0){char b[512];std::snprintf(b,sizeof(b),"F4X3:%s:%s:%d:%u:%u:%u:%d:%s",d,g,schema,session,seq,age,int(p.size())+len_adjust,p.c_str());std::string body=b;char out[600];std::snprintf(out,sizeof(out),"%s:%08X",body.c_str(),crc32(body));return out;}
int main(){
  VehicleTelemetry t=defaultTelemetry();
  auto a=parseExtendedTelemetryLine(v3("ESC","PWR",42,1,20,"1,51.2,12.3,4.1,11.8,0.2,0.2500,42.0,0").c_str(),t,1000);
  assert(a.recognized&&a.accepted&&a.v3&&t.escx.power.session==42);
  auto dup=parseExtendedTelemetryLine(v3("ESC","PWR",42,1,21,"1,52,1,1,1,1,0.1,40,0").c_str(),t,1010);
  assert(dup.recognized&&!dup.accepted&&dup.outOfOrder);
  std::string corrupt=v3("ESC","PWR",42,2,20,"1,53,1,1,1,1,0.1,40,0"); corrupt.back()=corrupt.back()=='0'?'1':'0';
  auto c=parseExtendedTelemetryLine(corrupt.c_str(),t,1020); assert(c.recognized&&!c.accepted&&c.crcError);
  auto l=parseExtendedTelemetryLine(v3("ESC","PWR",42,2,20,"1,53,1,1,1,1,0.1,40,0",3,1).c_str(),t,1030); assert(!l.accepted&&l.lengthError);
  auto ver=parseExtendedTelemetryLine(v3("ESC","PWR",42,2,20,"1,53,1,1,1,1,0.1,40,0",4,0).c_str(),t,1040); assert(!ver.accepted&&ver.versionError);
  auto news=parseExtendedTelemetryLine(v3("ESC","PWR",77,1,10,"1,54,1,1,1,1,0.1,40,0").c_str(),t,1050); assert(news.accepted&&t.escx.power.session==77&&t.escx.power.seq==1);
  auto legacy_fresh=parseExtendedTelemetryLine("ESCX:PWR:99:5:1,55,1,1,1,1,0.1,40,0",t,1060); assert(!legacy_fresh.accepted&&legacy_fresh.outOfOrder);
  auto legacy_late=parseExtendedTelemetryLine("ESCX:PWR:100:5:1,56,1,1,1,1,0.1,40,0",t,7000); assert(legacy_late.accepted&&t.escx.power.session==0);
  auto trunc=parseExtendedTelemetryLine("F4X3:ESC:PWR:3:1:1:0:5:1,2:1234",t,7010); assert(trunc.recognized&&!trunc.accepted);
  std::cout<<"TELEMETRY_V3_SELF_CHECK_PASS\n"; return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
    src=Path(td)/'check.cpp'; exe=Path(td)/'check'; src.write_text(CPP)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT/'src'),str(src),str(ROOT/'src/TelemetryProtocol.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS F4X3 CRC/length/version/session/legacy fault contract')
