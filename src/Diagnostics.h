// ============================================================================
// Diagnostics.h — local HMI/gateway health snapshot for read-only diagnostics.
// ============================================================================
#pragma once

#include <cstdint>

struct HmiDiagnostics {
  uint32_t uptimeMs{0};
  uint32_t hostCommands{0};
  uint32_t unknownCommands{0};
  uint32_t overlongCommands{0};
  uint32_t deferredCommandDrops{0};
  uint32_t deferredCommandPeak{0};
  uint32_t extendedTelemetryAccepted{0};
  uint32_t extendedTelemetryMalformed{0};
  uint32_t extendedTelemetryOutOfOrder{0};
  uint32_t uiFrames{0};
  uint32_t fullUiFrames{0};
  uint32_t dirtyUiFrames{0};
  uint32_t touchActions{0};
  uint32_t uiDrawLastMs{0};
  uint32_t uiDrawMaxMs{0};
  uint32_t maxServiceGapMs{0};
  uint32_t serviceGapLifetimeMaxMs{0};
  uint32_t serviceGapP95Ms{0};
  uint32_t serviceGapP99Ms{0};
  uint32_t stackHeadroomBytes{0};
  uint32_t minStackHeadroomBytes{0};
  uint32_t displayBytesLastFrame{0};
  uint32_t displayBytesMaxFrame{0};
  uint32_t spiTransactions{0};
  uint32_t spiBytesTx{0};
  uint32_t spiTimeoutCount{0};
  uint32_t spiHalErrorCount{0};
  uint32_t spiRecoveryCount{0};
  uint32_t spiBusConflictCount{0};
  uint32_t tftWriteClockHz{0};
  bool tftFastWriteValidated{false};
  bool tftUltraFastWriteValidated{false};
  uint32_t touchReadCount{0};
  uint32_t touchRejectFastCount{0};
  uint32_t lastHostCommandMs{0};
  uint32_t rosHeartbeatAgeMs{0xFFFFFFFFUL};
  uint32_t tftControllerId{0};
  uint8_t tftPowerMode{0};
  uint8_t tftMadctl{0};
  uint8_t tftPixelFormat{0};

  uint32_t vescUartErrors{0};
  uint32_t vescUartOverflow{0};
  uint32_t vescUartTxDropped{0};
  uint32_t gnssUartErrors{0};
  uint32_t gnssUartOverflow{0};
  uint32_t gnssUartTxDropped{0};
  uint32_t magErrors{0};
  uint32_t vescFrameErrors{0};
  uint32_t vescRecoveryCount{0};
  uint32_t vescLastFrameAgeMs{0xFFFFFFFFUL};

  bool tftOk{false};
  bool displayReady{false};
  bool displayFaulted{false};
  bool vescUartOk{false};
  bool gnssUartOk{false};
  bool magOk{false};
  bool pb6VescTx{false};
  bool pb7VescRx{false};
  bool pa2GnssTx{false};
  bool pa3GnssRx{false};
  bool pb8I2cScl{false};
  bool pb9I2cSda{false};
  bool pb12Safety{false};
  bool pb13SafetyLed{false};
  bool pa8Buzzer{false};
  bool pa5SpiSck{false};
  bool pa6SpiMiso{false};
  bool pa7SpiMosi{false};
  bool pb0TftCs{false};
  bool pb1TftDc{false};
  bool pb2TftRst{false};
  bool pa4TouchCs{false};
  bool pa11UsbDm{false};
  bool pa12UsbDp{false};
};

extern HmiDiagnostics gDiagnostics;
