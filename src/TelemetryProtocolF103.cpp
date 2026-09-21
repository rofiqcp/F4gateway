#include "TelemetryProtocol.h"
#include "NumericParse.h"

#if defined(BOARD_F103C8)
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace {
constexpr size_t kMaxLine = 360U;
constexpr size_t kMaxFields = 10U;

uint32_t crc32Bytes(const char *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0U; i < len; ++i) {
    crc ^= static_cast<uint8_t>(data[i]);
    for (uint8_t b = 0U; b < 8U; ++b)
      crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
  }
  return crc ^ 0xFFFFFFFFUL;
}

bool bool01(const char *s, bool &out) {
  uint32_t v = 0U;
  if (!CompactParse::u32(s, v) || v > 1U) return false;
  out = v != 0U;
  return true;
}

bool safeToken(const char *src, char *dst, size_t dstLen) {
  if (src == nullptr || dst == nullptr || dstLen < 2U) return false;
  size_t n = 0U;
  while (src[n] != '\0') {
    const char c = src[n];
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
    if (!ok || n + 1U >= dstLen) return false;
    dst[n] = c;
    ++n;
  }
  if (n == 0U) return false;
  dst[n] = '\0';
  return true;
}

void copyToken(char *dst, size_t dstLen, const char *src) {
  if (dst == nullptr || dstLen == 0U) return;
  size_t n = 0U;
  while (src != nullptr && src[n] != '\0' && n + 1U < dstLen) {
    dst[n] = src[n];
    ++n;
  }
  dst[n] = '\0';
}

size_t splitCsv(char *payload, char *fields[], size_t maxFields) {
  if (payload == nullptr || *payload == '\0') return 0U;
  size_t count = 0U;
  char *p = payload;
  while (count < maxFields) {
    fields[count++] = p;
    char *comma = std::strchr(p, ',');
    if (comma == nullptr) break;
    *comma = '\0';
    p = comma + 1;
  }
  if (count == maxFields && std::strchr(p, ',') != nullptr) return maxFields + 1U;
  return count;
}

bool acceptSequence(const TelemetryGroupStamp &stamp, uint32_t session,
                    uint32_t seq, uint32_t now) {
  if (stamp.lastRxMs == 0U) return true;
  if (session != 0U && stamp.session != session) return true;
  if (static_cast<uint32_t>(now - stamp.lastRxMs) > 5000U) return true;
  return static_cast<int32_t>(seq - stamp.seq) > 0;
}

void commitStamp(TelemetryGroupStamp &stamp, uint32_t session, uint32_t seq,
                 uint32_t age, bool valid, uint32_t now) {
  stamp.session = session;
  stamp.seq = seq;
  stamp.sourceAgeMs = age;
  stamp.lastRxMs = now;
  stamp.valid = valid && age != 0xFFFFFFFFUL;
  stamp.fresh = false;
}

bool parseCamera(char *payload, uint32_t session, uint32_t seq, uint32_t age,
                 VehicleTelemetry &t, uint32_t now, bool &ooo,
                 bool &sessionChanged) {
  char *f[kMaxFields]{};
  const size_t n = splitCsv(payload, f, kMaxFields);
  if (n != 8U) return false;
  bool valid=false, healthy=false, infer=false;
  float fps=0.0F, latency=0.0F;
  uint32_t dropped=0U, recovery=0U;
  char backend[12]{};
  if (!bool01(f[0],valid) || !bool01(f[1],healthy) ||
      !CompactParse::realf(f[2],fps) || !CompactParse::realf(f[3],latency) ||
      !CompactParse::u32(f[4],dropped) || !CompactParse::u32(f[5],recovery) ||
      !safeToken(f[6],backend,sizeof(backend)) || !bool01(f[7],infer)) return false;
  const uint32_t previous = t.perx.camera.session;
  if (!acceptSequence(t.perx.camera,session,seq,now)) { ooo=true; return false; }
  commitStamp(t.perx.camera,session,seq,age,valid,now);
  sessionChanged = previous != 0U && previous != session;
  if (valid) {
    t.perx.inferenceLatencyMs = std::clamp(latency,0.0F,10000.0F);
    t.perx.droppedFrames = dropped;
    t.perx.recoveryCount = recovery;
    copyToken(t.perx.backend,sizeof(t.perx.backend),backend);
    t.cameraFps = std::clamp(fps,0.0F,240.0F);
    t.cameraReady = healthy;
    t.perceptionInference = infer;
  }
  return true;
}

bool parseImu(char *payload, uint32_t session, uint32_t seq, uint32_t age,
              VehicleTelemetry &t, uint32_t now, bool &ooo,
              bool &sessionChanged) {
  char *f[kMaxFields]{};
  const size_t n = splitCsv(payload,f,kMaxFields);
  if (n != 4U) return false;
  bool valid=false;
  float yaw=0.0F, fused=0.0F, disagreement=0.0F;
  if (!bool01(f[0],valid) || !CompactParse::realf(f[1],yaw) ||
      !CompactParse::realf(f[2],fused) || !CompactParse::realf(f[3],disagreement)) return false;
  const uint32_t previous = t.navx.imuMag.session;
  if (!acceptSequence(t.navx.imuMag,session,seq,now)) { ooo=true; return false; }
  commitStamp(t.navx.imuMag,session,seq,age,valid,now);
  sessionChanged = previous != 0U && previous != session;
  if (valid) {
    t.navx.imuYawDeg = yaw;
    t.navx.fusedHeadingDeg = fused;
    t.navx.headingDisagreementDeg = std::clamp(disagreement,0.0F,180.0F);
  }
  return true;
}

bool parseNav2(char *payload, uint32_t session, uint32_t seq, uint32_t age,
               VehicleTelemetry &t, uint32_t now, bool &ooo,
               bool &sessionChanged) {
  char *f[kMaxFields]{};
  const size_t n = splitCsv(payload,f,kMaxFields);
  if (n != 9U) return false;
  bool valid=false, ready=false, path=false;
  uint32_t cmdAge=0U;
  float lin=0.0F, ang=0.0F;
  char planner[12]{}, controller[12]{}, smoother[12]{};
  if (!bool01(f[0],valid) || !bool01(f[1],ready) || !bool01(f[2],path) ||
      !CompactParse::u32(f[3],cmdAge) || !CompactParse::realf(f[4],lin) ||
      !CompactParse::realf(f[5],ang) || !safeToken(f[6],planner,sizeof(planner)) ||
      !safeToken(f[7],controller,sizeof(controller)) ||
      !safeToken(f[8],smoother,sizeof(smoother))) return false;
  const uint32_t previous = t.navx.nav2.session;
  if (!acceptSequence(t.navx.nav2,session,seq,now)) { ooo=true; return false; }
  commitStamp(t.navx.nav2,session,seq,age,valid,now);
  sessionChanged = previous != 0U && previous != session;
  if (valid) {
    t.navx.nav2StackReady = ready;
    t.navx.pathValid = path;
    t.navx.cmdAgeMs = cmdAge;
    t.navx.commandLinearMps = lin;
    t.navx.commandAngularRps = ang;
    copyToken(t.navx.plannerState,sizeof(t.navx.plannerState),planner);
    copyToken(t.navx.controllerState,sizeof(t.navx.controllerState),controller);
    copyToken(t.navx.smootherState,sizeof(t.navx.smootherState),smoother);
  }
  return true;
}
} // namespace

ExtendedTelemetryParseResult parseExtendedTelemetryLine(
    const char *line, VehicleTelemetry &telemetry, uint32_t nowMs,
    uint32_t expectedSession) {
  ExtendedTelemetryParseResult result{};
  if (line == nullptr || std::strncmp(line,"F4X3:",5) != 0) return result;
  result.recognized = true;
  result.v3 = true;

  const size_t len = std::strlen(line);
  if (len == 0U || len > kMaxLine) { result.lengthError=true; return result; }
  const char *crcSep = std::strrchr(line, ':');
  if (crcSep == nullptr || crcSep <= line + 5) { result.lengthError=true; return result; }
  uint32_t expectedCrc=0U;
  if (!CompactParse::hex32(crcSep+1,expectedCrc)) { result.crcError=true; return result; }
  const size_t signedLen = static_cast<size_t>(crcSep-line);
  if (crc32Bytes(line,signedLen) != expectedCrc) { result.crcError=true; return result; }

  char buf[kMaxLine+1U];
  std::memcpy(buf,line,signedLen);
  buf[signedLen]='\0';
  char *parts[9]{};
  size_t count=0U;
  char *cur=buf;
  while (count < 8U) {
    parts[count++]=cur;
    char *colon=std::strchr(cur,':');
    if (colon == nullptr) { result.lengthError=true; return result; }
    *colon='\0';
    cur=colon+1;
  }
  parts[count++]=cur;
  if (count != 9U || std::strcmp(parts[0],"F4X3") != 0) {
    result.lengthError=true; return result;
  }

  if (std::strcmp(parts[1],"PER")==0) result.domain=ExtendedTelemetryDomain::PERCEPTION;
  else if (std::strcmp(parts[1],"NAV")==0) result.domain=ExtendedTelemetryDomain::NAVIGATION;
  else if (std::strcmp(parts[1],"ESC")==0) result.domain=ExtendedTelemetryDomain::ESC;
  else return result;

  uint32_t schema=0U, session=0U, seq=0U, age=0U, payloadLen=0U;
  if (!CompactParse::u32(parts[3],schema) || schema != 3U) {
    result.versionError=true; return result;
  }
  if (!CompactParse::u32(parts[4],session) || session==0U ||
      !CompactParse::u32(parts[5],seq) || !CompactParse::u32(parts[6],age) ||
      !CompactParse::u32(parts[7],payloadLen)) return result;
  if (expectedSession != 0U && session != expectedSession) {
    result.sessionError=true; return result;
  }
  if (std::strlen(parts[8]) != payloadLen) { result.lengthError=true; return result; }

  bool ooo=false, changed=false, ok=false;
  if (result.domain == ExtendedTelemetryDomain::PERCEPTION &&
      std::strcmp(parts[2],"CAM")==0)
    ok=parseCamera(parts[8],session,seq,age,telemetry,nowMs,ooo,changed);
  else if (result.domain == ExtendedTelemetryDomain::NAVIGATION &&
           std::strcmp(parts[2],"IMU")==0)
    ok=parseImu(parts[8],session,seq,age,telemetry,nowMs,ooo,changed);
  else if (result.domain == ExtendedTelemetryDomain::NAVIGATION &&
           std::strcmp(parts[2],"NAV2")==0)
    ok=parseNav2(parts[8],session,seq,age,telemetry,nowMs,ooo,changed);

  result.accepted=ok;
  result.outOfOrder=ooo;
  result.sessionChanged=ok && changed;
  return result;
}
#endif // BOARD_F103C8
