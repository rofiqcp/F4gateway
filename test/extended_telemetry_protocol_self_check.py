#!/usr/bin/env python3
from pathlib import Path
import subprocess, tempfile, textwrap, sys
ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include "TelemetryProtocol.h"
int main() {
  VehicleTelemetry t=defaultTelemetry();
  auto p=parseExtendedTelemetryLine("ESCX:PWR:1:20:1,51.2,12.3,4.1,11.8,0.2,0.2500,42.0,0",t,1000);
  assert(p.recognized && p.accepted && !p.outOfOrder);
  assert(std::fabs(t.escx.vbusV-51.2f)<0.01f && t.escx.faultCode==0);
  auto dup=parseExtendedTelemetryLine("ESCX:PWR:1:21:1,52.0,1,1,1,1,0.1,40,0",t,1010);
  assert(dup.recognized && !dup.accepted && dup.outOfOrder);
  const float old=t.escx.vbusV;
  auto bad=parseExtendedTelemetryLine("ESCX:PWR:2:10:1,nan,1,1,1,1,0.1,40,0",t,1020);
  assert(bad.recognized && !bad.accepted && !bad.outOfOrder && t.escx.vbusV==old);
  auto m=parseExtendedTelemetryLine("ESCX:MTR:1:15:1,51.1,2.0,0.1,100,0,51.3,5.0,0.2,300,0",t,1030);
  assert(m.accepted && std::fabs(t.escx.rightRpm-300.0f)<0.1f);
  auto e=parseExtendedTelemetryLine("ESCX:ENC:1:8:1,123,1000,500,1,1,1,0,2.5",t,1040);
  assert(e.accepted && t.escx.encoderRaw==123 && t.escx.encoderSynced);
  auto c=parseExtendedTelemetryLine("PERX:CAM:1:10:1,1,12.5,8.2,3,0,GPU,1",t,1050);
  assert(c.accepted && t.cameraReady && t.perceptionInference);
  auto d=parseExtendedTelemetryLine("PERX:DET:1:12:1,2,2,0,7,1.25,-0.20,88.0,PERSON",t,1060);
  assert(d.accepted && t.perx.objectCount==2 && std::strcmp(t.detectedObject,"PERSON")==0);
  auto l=parseExtendedTelemetryLine("PERX:LANE:1:14:1,1,-0.05,91.0,1.2,0.3,0.4,2.0",t,1070);
  assert(l.accepted && t.perx.laneValid);
  auto pose=parseExtendedTelemetryLine("NAVX:POSE:1:18:1,1.2,-0.5,90,0.01,0.02,0.001",t,1080);
  assert(pose.accepted && std::fabs(t.navx.mapYawDeg-90.0f)<0.1f);
  auto nav=parseExtendedTelemetryLine("NAVX:NAV2:1:20:1,1,1,35,0.4,0.1,READY,READY,WAIT",t,1090);
  assert(nav.accepted && t.navx.pathValid && t.navx.cmdAgeMs==35);
  updateExtendedFreshness(t,1100);
  assert(t.escx.power.fresh && t.perx.camera.fresh && t.navx.pose.fresh);
  updateExtendedFreshness(t,5000);
  assert(!t.escx.power.fresh && !t.perx.camera.fresh && !t.navx.pose.fresh);
  std::string longline="ESCX:PWR:9:0:"+std::string(221,'1');
  auto over=parseExtendedTelemetryLine(longline.c_str(),t,6000);
  assert(over.recognized && !over.accepted);
  std::cout << "PASS EXTENDED_TELEMETRY_PROTOCOL" << std::endl;
  return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
    src=Path(td)/'check.cpp'; exe=Path(td)/'check'
    src.write_text(CPP)
    cmd=['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT/'src'),str(src),str(ROOT/'src/TelemetryProtocol.cpp'),'-o',str(exe)]
    subprocess.run(cmd,check=True)
    subprocess.run([str(exe)],check=True)
print('PASS STAGE2 protocol parser/freshness functional self-check')