#pragma once

#include "McuHal.h"
#include "FirmwareConfig.h"
#include <cstddef>
#include <cstdint>

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
#define htim11 htim3
#if BTS_WINCH_ENABLED
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
void Board_BtsSetPwm(uint16_t rpwm, uint16_t lpwm);
void Board_BtsEmergencyCut();
uint16_t Board_BtsDutyTicks();
int8_t Board_BtsDirection();
#endif

enum class BoardSpiOwner : uint8_t {
  NONE = 0U,
  HMI_TFT,
  HMI_TOUCH
};

bool Board_SpiAcquire(BoardSpiOwner owner, uint32_t prescaler);
void Board_SpiRelease(BoardSpiOwner owner);
void Board_SpiDeselectAll();
BoardSpiOwner Board_SpiOwner();
uint32_t Board_SpiContentionCount();
uint32_t Board_SpiRecoveryCount();

void Board_Init();
void Board_Service();
uint32_t Board_MaxServiceGapMs();
uint32_t Board_SteadyMaxServiceGapMs();
uint32_t Board_ServiceGapP95Ms();
uint32_t Board_ServiceGapP99Ms();
void Board_ResetServiceGapStats();
uint32_t Board_StackHeadroomBytes();
uint32_t Board_MinStackHeadroomBytes();
void Board_SetRealtimeServiceCallback(void (*callback)());
void Board_RealtimeService();
void Board_DelayUs(uint32_t microseconds);
void Board_RealtimeDelayMs(uint32_t duration_ms);
bool Board_ReinitSpi1();
#ifdef HMI_TEST_HOOKS
void Board_TestInjectSpiInitFailureOnce();
#endif
void Board_SetWatchdogCallback(void (*callback)());
void Board_WatchdogStart();
void Board_WatchdogStop();
void Board_BuzzerStart(uint16_t frequency_hz, uint16_t duration_ms);
void Board_BuzzerStop();
void Board_BackupWrite(uint8_t slot, uint32_t value);
uint32_t Board_BackupRead(uint8_t slot);
