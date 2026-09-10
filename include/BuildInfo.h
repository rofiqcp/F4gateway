#pragma once

#include <cstdint>

#ifndef FW_GIT_SHA
#define FW_GIT_SHA "unknown"
#endif
#ifndef FW_GIT_FULL_SHA
#define FW_GIT_FULL_SHA "unknown"
#endif
#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY 1
#endif
#ifndef FW_BUILD_EPOCH
#define FW_BUILD_EPOCH 0
#endif
#ifndef FW_SCHEMA_VERSION
#define FW_SCHEMA_VERSION 0
#endif

namespace BuildInfo {
inline constexpr const char* kGitSha = FW_GIT_SHA;
inline constexpr const char* kGitFullSha = FW_GIT_FULL_SHA;
inline constexpr bool kGitDirty = FW_GIT_DIRTY != 0;
inline constexpr uint32_t kBuildEpoch = static_cast<uint32_t>(FW_BUILD_EPOCH);
inline constexpr uint16_t kSchemaVersion = static_cast<uint16_t>(FW_SCHEMA_VERSION);
inline constexpr uint32_t kAppBase = 0x08008000UL;
inline constexpr uint32_t kAppLimit = 0x08060000UL;
}  // namespace BuildInfo
