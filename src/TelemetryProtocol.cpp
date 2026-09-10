#include "TelemetryProtocol.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace {
constexpr size_t kMaxExtendedLine = 220U;
constexpr size_t kMaxCsvFields = 20U;

bool parseU32(const char *s, uint32_t &out) {
  if (s == nullptr || *s == '\0') return false;
  errno = 0;
  char *end = nullptr;
  const unsigned long v = std::strtoul(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || v > 0xFFFFFFFFUL) return false;
  out = static_cast<uint32_t>(v);
  return true;
}

bool parseI32(const char *s, int32_t &out) {
  if (s == nullptr || *s == '\0') return false;
  errno = 0;
  char *end = nullptr;
  const long v = std::strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' ||
      v < std::numeric_limits<int32_t>::min() ||
      v > std::numeric_limits<int32_t>::max()) return false;
  out = static_cast<int32_t>(v);
  return true;
}

bool parseFloat(const char *s, float &out) {
  if (s == nullptr || *s == '\0') return false;
  errno = 0;
  char *end = nullptr;
  const float v = std::strtof(s, &end);
  if (errno != 0 || end == s || *end != '\0' || !std::isfinite(v)) return false;
  out = v;
  return true;
}

bool parseBool01(const char *s, bool &out) {
  uint32_t v = 0U;
  if (!parseU32(s, v) || v > 1U) return false;
  out = v != 0U;
  return true;
}

size_t splitCsv(char *payload, char *fields[], size_t maxFields) {
  if (payload == nullptr || *payload == '\0') return 0U;
  size_t count = 0U;
  char *cursor = payload;
  while (cursor != nullptr && count < maxFields) {
    fields[count++] = cursor;
    char *comma = std::strchr(cursor, ',');
    if (comma == nullptr) break;
    *comma = '\0';
    cursor = comma + 1;
  }
  if (cursor != nullptr && count == maxFields && std::strchr(cursor, ',') != nullptr)
    return maxFields + 1U;
  return count;
}

bool safeToken(const char *src, char *dst, size_t dstLen) {
  if (src == nullptr || dst == nullptr || dstLen < 2U) return false;
  size_t out = 0U;
  for (size_t i = 0U; src[i] != '\0'; ++i) {
    const char c = src[i];
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
    if (!ok || out + 1U >= dstLen) return false;
    dst[out++] = c;
  }
  if (out == 0U) return false;
  dst[out] = '\0';
  return true;
}

bool acceptSequence(const TelemetryGroupStamp &stamp, uint32_t seq, uint32_t now) {
  if (stamp.lastRxMs == 0U) return true;
  if (static_cast<uint32_t>(now - stamp.lastRxMs) > 5000U) return true;
  return static_cast<int32_t>(seq - stamp.seq) > 0;
}

void commitStamp(TelemetryGroupStamp &stamp, uint32_t seq, uint32_t age,
                 bool valid, uint32_t now) {
  stamp.seq = seq;
  stamp.sourceAgeMs = age;
  stamp.lastRxMs = now;
  stamp.valid = valid && age != 0xFFFFFFFFUL;
  stamp.fresh = false;  // recomputed centrally after parser returns
}

struct Header {
  ExtendedTelemetryDomain domain{ExtendedTelemetryDomain::NONE};
  char *group{nullptr};
  uint32_t seq{0U};
  uint32_t age{0xFFFFFFFFUL};
  char *payload{nullptr};
};

bool parseHeader(char *buf, Header &h) {
  char *p1 = std::strchr(buf, ':');
  if (!p1) return false;
  *p1 = '\0';
  if (std::strcmp(buf, "ESCX") == 0) h.domain = ExtendedTelemetryDomain::ESC;
  else if (std::strcmp(buf, "PERX") == 0) h.domain = ExtendedTelemetryDomain::PERCEPTION;
  else if (std::strcmp(buf, "NAVX") == 0) h.domain = ExtendedTelemetryDomain::NAVIGATION;
  else return false;
  h.group = p1 + 1;
  char *p2 = std::strchr(h.group, ':');
  if (!p2) return false;
  *p2 = '\0';
  char *seq = p2 + 1;
  char *p3 = std::strchr(seq, ':');
  if (!p3) return false;
  *p3 = '\0';
  char *age = p3 + 1;
  char *p4 = std::strchr(age, ':');
  if (!p4) return false;
  *p4 = '\0';
  h.payload = p4 + 1;
  return parseU32(seq, h.seq) && parseU32(age, h.age) && *h.payload != '\0';
}

bool parseEsc(const Header &h, char *f[], size_t n, VehicleTelemetry &t,
              uint32_t now, bool &outOfOrder) {
  if (std::strcmp(h.group, "PWR") == 0) {
    if (n != 9U) return false;
    bool valid = false; float vbus=0, motor=0, input=0, iq=0, id=0, duty=0, mos=0;
    uint32_t fault=0;
    if (!parseBool01(f[0], valid) || !parseFloat(f[1], vbus) ||
        !parseFloat(f[2], motor) || !parseFloat(f[3], input) ||
        !parseFloat(f[4], iq) || !parseFloat(f[5], id) ||
        !parseFloat(f[6], duty) || !parseFloat(f[7], mos) ||
        !parseU32(f[8], fault) || fault > 255U) return false;
    if (!acceptSequence(t.escx.power, h.seq, now)) { outOfOrder = true; return false; }
    commitStamp(t.escx.power, h.seq, h.age, valid, now);
    if (valid) {
      t.escx.vbusV = std::clamp(vbus, 0.0F, 100.0F);
      t.escx.motorCurrentA = std::clamp(motor, -500.0F, 500.0F);
      t.escx.inputCurrentA = std::clamp(input, -500.0F, 500.0F);
      t.escx.iqA = std::clamp(iq, -500.0F, 500.0F);
      t.escx.idA = std::clamp(id, -500.0F, 500.0F);
      t.escx.duty = std::clamp(duty, -1.0F, 1.0F);
      t.escx.mosTempC = std::clamp(mos, -50.0F, 200.0F);
      t.escx.faultCode = static_cast<uint8_t>(fault);
    }
    return true;
  }
  if (std::strcmp(h.group, "MTR") == 0) {
    if (n != 11U) return false;
    bool valid=false; float lv=0,li=0,ld=0,lr=0,rv=0,ri=0,rd=0,rr=0; uint32_t lf=0,rf=0;
    if (!parseBool01(f[0], valid) || !parseFloat(f[1], lv) || !parseFloat(f[2], li) ||
        !parseFloat(f[3], ld) || !parseFloat(f[4], lr) || !parseU32(f[5], lf) ||
        !parseFloat(f[6], rv) || !parseFloat(f[7], ri) || !parseFloat(f[8], rd) ||
        !parseFloat(f[9], rr) || !parseU32(f[10], rf) || lf > 255U || rf > 255U) return false;
    if (!acceptSequence(t.escx.motors, h.seq, now)) { outOfOrder = true; return false; }
    commitStamp(t.escx.motors, h.seq, h.age, valid, now);
    if (valid) {
      t.escx.leftVbusV=lv; t.escx.leftCurrentA=li; t.escx.leftDuty=ld; t.escx.leftRpm=lr;
      t.escx.leftFault=static_cast<uint8_t>(lf); t.escx.rightVbusV=rv; t.escx.rightCurrentA=ri;
      t.escx.rightDuty=rd; t.escx.rightRpm=rr; t.escx.rightFault=static_cast<uint8_t>(rf);
    }
    return true;
  }
  if (std::strcmp(h.group, "ENC") == 0) {
    if (n != 9U) return false;
    bool valid=false,cal=false,homed=false,sync=false,inv=false; int32_t raw=0,span=0,target=0; float pos=0;
    if (!parseBool01(f[0], valid) || !parseI32(f[1], raw) || !parseI32(f[2], span) ||
        !parseI32(f[3], target) || !parseBool01(f[4], cal) || !parseBool01(f[5], homed) ||
        !parseBool01(f[6], sync) || !parseBool01(f[7], inv) || !parseFloat(f[8], pos)) return false;
    if (!acceptSequence(t.escx.encoder, h.seq, now)) { outOfOrder = true; return false; }
    commitStamp(t.escx.encoder, h.seq, h.age, valid, now);
    if (valid) {
      t.escx.encoderRaw=raw; t.escx.encoderSpan=span; t.escx.encoderTarget=target;
      t.escx.calibrated=cal; t.escx.homed=homed; t.escx.encoderSynced=sync;
      t.escx.encoderInverted=inv; t.escx.encoderPositionDeg=std::clamp(pos,-180.0F,180.0F);
    }
    return true;
  }
  if (std::strcmp(h.group, "LINK") == 0) {
    if (n != 3U) return false;
    bool valid=false; uint32_t baud=0; char owner[12]{};
    if (!parseBool01(f[0], valid) || !parseU32(f[1], baud) ||
        !safeToken(f[2], owner, sizeof(owner))) return false;
    if (!acceptSequence(t.escx.link, h.seq, now)) { outOfOrder = true; return false; }
    commitStamp(t.escx.link, h.seq, h.age, valid, now);
    if (valid) { t.escx.uartBaud=baud; std::snprintf(t.escx.owner,sizeof(t.escx.owner),"%s",owner); }
    return true;
  }
  if (std::strcmp(h.group, "PERF") == 0) {
    if (n != 3U) return false;
    bool valid=false; uint32_t ca=0,fa=0;
    if (!parseBool01(f[0], valid) || !parseU32(f[1], ca) || !parseU32(f[2], fa)) return false;
    if (!acceptSequence(t.escx.performance, h.seq, now)) { outOfOrder = true; return false; }
    commitStamp(t.escx.performance, h.seq, h.age, valid, now);
    if (valid) { t.escx.commandAgeMs=ca; t.escx.feedbackAgeMs=fa; }
    return true;
  }
  return false;
}

bool parsePer(const Header &h, char *f[], size_t n, VehicleTelemetry &t,
              uint32_t now, bool &outOfOrder) {
  if (std::strcmp(h.group, "CAM") == 0) {
    if (n != 8U) return false;
    bool valid=false,healthy=false,infer=false; float fps=0,lat=0; uint32_t dropped=0,recovery=0; char backend[12]{};
    if (!parseBool01(f[0],valid)||!parseBool01(f[1],healthy)||!parseFloat(f[2],fps)||
        !parseFloat(f[3],lat)||!parseU32(f[4],dropped)||!parseU32(f[5],recovery)||
        !safeToken(f[6],backend,sizeof(backend))||!parseBool01(f[7],infer)) return false;
    if (!acceptSequence(t.perx.camera,h.seq,now)) { outOfOrder=true; return false; }
    commitStamp(t.perx.camera,h.seq,h.age,valid,now);
    if (valid) { t.perx.inferenceLatencyMs=std::clamp(lat,0.0F,10000.0F); t.perx.droppedFrames=dropped;
      t.perx.recoveryCount=recovery; std::snprintf(t.perx.backend,sizeof(t.perx.backend),"%s",backend);
      t.cameraFps=std::clamp(fps,0.0F,240.0F); t.cameraReady=healthy; t.perceptionInference=infer; }
    return true;
  }
  if (std::strcmp(h.group, "DET") == 0) {
    if (n != 9U) return false;
    bool valid=false; uint32_t count=0,tracks=0,missed=0; int32_t track=-1; float dist=0,lat=0,conf=0; char cls[24]{};
    if(!parseBool01(f[0],valid)||!parseU32(f[1],count)||!parseU32(f[2],tracks)||!parseU32(f[3],missed)||
       !parseI32(f[4],track)||!parseFloat(f[5],dist)||!parseFloat(f[6],lat)||!parseFloat(f[7],conf)||
       !safeToken(f[8],cls,sizeof(cls))) return false;
    if (!acceptSequence(t.perx.detection,h.seq,now)) { outOfOrder=true; return false; }
    commitStamp(t.perx.detection,h.seq,h.age,valid,now);
    if(valid){ t.perx.objectCount=static_cast<uint16_t>(std::min<uint32_t>(count,65535U));
      t.perx.trackCount=static_cast<uint16_t>(std::min<uint32_t>(tracks,65535U)); t.perx.missedTracks=static_cast<uint16_t>(std::min<uint32_t>(missed,65535U));
      t.perx.nearestTrackId=track; t.perx.nearestLateralM=std::clamp(lat,-100.0F,100.0F);
      t.objectDistanceM=std::max(0.0F,dist); t.confidencePct=std::clamp(conf,0.0F,100.0F);
      std::snprintf(t.detectedObject,sizeof(t.detectedObject),"%s",cls); }
    return true;
  }
  if (std::strcmp(h.group, "LANE") == 0) {
    if (n != 8U) return false;
    bool valid=false,laneValid=false; float center=0,conf=0,width=0,left=0,right=0,heading=0;
    if(!parseBool01(f[0],valid)||!parseBool01(f[1],laneValid)||!parseFloat(f[2],center)||!parseFloat(f[3],conf)||
       !parseFloat(f[4],width)||!parseFloat(f[5],left)||!parseFloat(f[6],right)||!parseFloat(f[7],heading)) return false;
    if(!acceptSequence(t.perx.lane,h.seq,now)){outOfOrder=true;return false;}
    commitStamp(t.perx.lane,h.seq,h.age,valid,now);
    if(valid){t.perx.laneValid=laneValid;t.perx.laneCenterOffsetM=center;t.perx.laneConfidencePct=std::clamp(conf,0.0F,100.0F);
      t.perx.roadWidthM=std::max(0.0F,width);t.perx.laneLeftClearanceM=left;t.perx.laneRightClearanceM=right;t.perx.laneHeadingErrorDeg=heading;}
    return true;
  }
  if (std::strcmp(h.group, "DRV") == 0) {
    if (n != 6U) return false;
    bool valid=false,drvValid=false; float fraction=0,left=0,right=0,far=0;
    if(!parseBool01(f[0],valid)||!parseBool01(f[1],drvValid)||!parseFloat(f[2],fraction)||!parseFloat(f[3],left)||!parseFloat(f[4],right)||!parseFloat(f[5],far)) return false;
    if(!acceptSequence(t.perx.drivable,h.seq,now)){outOfOrder=true;return false;}
    commitStamp(t.perx.drivable,h.seq,h.age,valid,now);
    if(valid){t.perx.drivableValid=drvValid;t.perx.drivableFractionPct=std::clamp(fraction,0.0F,100.0F);t.perx.drivableLeftM=left;t.perx.drivableRightM=right;t.perx.farLookaheadM=std::max(0.0F,far);}
    return true;
  }
  if (std::strcmp(h.group, "OBS") == 0) {
    if (n != 8U) return false;
    bool valid=false,blocked=false; uint32_t count=0,missed=0; int32_t track=-1; float dist=0,lat=0;
    if(!parseBool01(f[0],valid)||!parseU32(f[1],count)||!parseBool01(f[2],blocked)||!parseFloat(f[3],dist)||!parseFloat(f[4],lat)||!parseI32(f[5],track)||!parseU32(f[6],missed)) return false;
    uint32_t reserved=0; if(!parseU32(f[7],reserved)) return false;
    if(!acceptSequence(t.perx.obstacle,h.seq,now)){outOfOrder=true;return false;}
    commitStamp(t.perx.obstacle,h.seq,h.age,valid,now);
    if(valid){t.perx.obstacleCount=static_cast<uint16_t>(std::min<uint32_t>(count,65535U));t.perx.obstacleValid=true;t.perx.obstacleBlocked=blocked;
      t.perx.obstacleDistanceM=std::max(0.0F,dist);t.perx.obstacleLateralM=lat;t.perx.nearestTrackId=track;t.perx.missedTracks=static_cast<uint16_t>(std::min<uint32_t>(missed,65535U));t.obstacleDetected=blocked;}
    return true;
  }
  if (std::strcmp(h.group, "PERF") == 0) {
    if (n != 7U) return false;
    bool valid=false; float fps=0,lat=0; uint32_t dropped=0,raw=0,confirmed=0; char backend[12]{};
    if(!parseBool01(f[0],valid)||!parseFloat(f[1],fps)||!parseFloat(f[2],lat)||!parseU32(f[3],dropped)||!parseU32(f[4],raw)||!parseU32(f[5],confirmed)||!safeToken(f[6],backend,sizeof(backend))) return false;
    if(!acceptSequence(t.perx.performance,h.seq,now)){outOfOrder=true;return false;}
    commitStamp(t.perx.performance,h.seq,h.age,valid,now);
    if(valid){t.cameraFps=std::clamp(fps,0.0F,240.0F);t.perx.inferenceLatencyMs=std::clamp(lat,0.0F,10000.0F);t.perx.droppedFrames=dropped;
      t.perx.rawDetectionCount=raw;t.perx.confirmedObstacleCount=confirmed;std::snprintf(t.perx.backend,sizeof(t.perx.backend),"%s",backend);}
    return true;
  }
  return false;
}

bool parseNav(const Header &h, char *f[], size_t n, VehicleTelemetry &t,
              uint32_t now, bool &outOfOrder) {
  if (std::strcmp(h.group, "POSE") == 0) {
    if (n != 7U) return false;
    bool valid=false; float x=0,y=0,yaw=0,cx=0,cy=0,yv=0;
    if(!parseBool01(f[0],valid)||!parseFloat(f[1],x)||!parseFloat(f[2],y)||!parseFloat(f[3],yaw)||!parseFloat(f[4],cx)||!parseFloat(f[5],cy)||!parseFloat(f[6],yv))return false;
    if(!acceptSequence(t.navx.pose,h.seq,now)){outOfOrder=true;return false;} commitStamp(t.navx.pose,h.seq,h.age,valid,now);
    if(valid){t.navx.mapX=x;t.navx.mapY=y;t.navx.mapYawDeg=yaw;t.navx.covarianceX=std::max(0.0F,cx);t.navx.covarianceY=std::max(0.0F,cy);t.navx.yawVariance=std::max(0.0F,yv);} return true;
  }
  if (std::strcmp(h.group, "ODOM") == 0) {
    if (n != 6U) return false;
    bool valid=false; float x=0,y=0,yaw=0,lin=0,yr=0;
    if(!parseBool01(f[0],valid)||!parseFloat(f[1],x)||!parseFloat(f[2],y)||!parseFloat(f[3],yaw)||!parseFloat(f[4],lin)||!parseFloat(f[5],yr))return false;
    if(!acceptSequence(t.navx.odom,h.seq,now)){outOfOrder=true;return false;} commitStamp(t.navx.odom,h.seq,h.age,valid,now);
    if(valid){t.navx.odomX=x;t.navx.odomY=y;t.navx.odomYawDeg=yaw;t.navx.odomLinearMps=lin;t.navx.odomYawRateRps=yr;} return true;
  }
  if (std::strcmp(h.group, "IMU") == 0) {
    if (n != 4U) return false;
    bool valid=false; float iy=0,fh=0,dis=0;
    if(!parseBool01(f[0],valid)||!parseFloat(f[1],iy)||!parseFloat(f[2],fh)||!parseFloat(f[3],dis))return false;
    if(!acceptSequence(t.navx.imuMag,h.seq,now)){outOfOrder=true;return false;} commitStamp(t.navx.imuMag,h.seq,h.age,valid,now);
    if(valid){t.navx.imuYawDeg=iy;t.navx.fusedHeadingDeg=fh;t.navx.headingDisagreementDeg=std::clamp(dis,0.0F,180.0F);} return true;
  }
  if (std::strcmp(h.group, "NAV2") == 0) {
    if (n != 9U) return false;
    bool valid=false,ready=false,path=false; uint32_t age=0; float lin=0,ang=0; char planner[12]{},controller[12]{},smoother[12]{};
    if(!parseBool01(f[0],valid)||!parseBool01(f[1],ready)||!parseBool01(f[2],path)||!parseU32(f[3],age)||!parseFloat(f[4],lin)||!parseFloat(f[5],ang)||
       !safeToken(f[6],planner,sizeof(planner))||!safeToken(f[7],controller,sizeof(controller))||!safeToken(f[8],smoother,sizeof(smoother)))return false;
    if(!acceptSequence(t.navx.nav2,h.seq,now)){outOfOrder=true;return false;} commitStamp(t.navx.nav2,h.seq,h.age,valid,now);
    if(valid){t.navx.nav2StackReady=ready;t.navx.pathValid=path;t.navx.cmdAgeMs=age;t.navx.commandLinearMps=lin;t.navx.commandAngularRps=ang;
      std::snprintf(t.navx.plannerState,sizeof(t.navx.plannerState),"%s",planner);std::snprintf(t.navx.controllerState,sizeof(t.navx.controllerState),"%s",controller);std::snprintf(t.navx.smootherState,sizeof(t.navx.smootherState),"%s",smoother);} return true;
  }
  if (std::strcmp(h.group, "COST") == 0) {
    if (n != 5U) return false;
    bool valid=false,ready=false,blocked=false; uint32_t obs=0,rel=0;
    if(!parseBool01(f[0],valid)||!parseBool01(f[1],ready)||!parseU32(f[2],obs)||!parseU32(f[3],rel)||!parseBool01(f[4],blocked))return false;
    if(!acceptSequence(t.navx.costmap,h.seq,now)){outOfOrder=true;return false;} commitStamp(t.navx.costmap,h.seq,h.age,valid,now);
    if(valid){t.navx.costmapReady=ready;t.navx.obstaclePointCount=static_cast<uint16_t>(std::min<uint32_t>(obs,65535U));t.navx.pathRelevantCount=static_cast<uint16_t>(std::min<uint32_t>(rel,65535U));t.navx.costmapBlocked=blocked;} return true;
  }
  if (std::strcmp(h.group, "CTRL") == 0) {
    if (n != 4U) return false;
    bool valid=false; uint32_t age=0; float lin=0,ang=0;
    if(!parseBool01(f[0],valid)||!parseU32(f[1],age)||!parseFloat(f[2],lin)||!parseFloat(f[3],ang))return false;
    if(!acceptSequence(t.navx.control,h.seq,now)){outOfOrder=true;return false;} commitStamp(t.navx.control,h.seq,h.age,valid,now);
    if(valid){t.navx.cmdAgeMs=age;t.navx.commandLinearMps=lin;t.navx.commandAngularRps=ang;} return true;
  }
  return false;
}
}  // namespace

ExtendedTelemetryParseResult parseExtendedTelemetryLine(
    const char *line, VehicleTelemetry &telemetry, uint32_t nowMs) {
  ExtendedTelemetryParseResult result{};
  if (line == nullptr) return result;
  if (std::strncmp(line,"ESCX:",5)!=0 && std::strncmp(line,"PERX:",5)!=0 &&
      std::strncmp(line,"NAVX:",5)!=0) return result;
  result.recognized = true;
  const size_t len = std::strlen(line);
  if (len == 0U || len > kMaxExtendedLine) return result;
  char buf[kMaxExtendedLine + 1U];
  std::memcpy(buf,line,len+1U);
  Header h{};
  if (!parseHeader(buf,h)) return result;
  result.domain = h.domain;
  char *fields[kMaxCsvFields]{};
  const size_t count = splitCsv(h.payload,fields,kMaxCsvFields);
  if (count == 0U || count > kMaxCsvFields) return result;
  bool outOfOrder=false;
  bool ok=false;
  if(h.domain==ExtendedTelemetryDomain::ESC) ok=parseEsc(h,fields,count,telemetry,nowMs,outOfOrder);
  else if(h.domain==ExtendedTelemetryDomain::PERCEPTION) ok=parsePer(h,fields,count,telemetry,nowMs,outOfOrder);
  else if(h.domain==ExtendedTelemetryDomain::NAVIGATION) ok=parseNav(h,fields,count,telemetry,nowMs,outOfOrder);
  result.accepted=ok;
  result.outOfOrder=outOfOrder;
  return result;
}
