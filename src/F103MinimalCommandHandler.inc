static void handleSerialCommandF103(char *command) {
  while (*command == ' ' || *command == '\t') ++command;
  if (*command == '\0') return;
  if (gRealtimeParserActive) {
    (void)enqueueDeferredCommand(command);
    return;
  }
  ++gDiagnostics.hostCommands;
  gDiagnostics.lastHostCommandMs = HAL_GetTick();

  if (!std::strncmp(command, "HOST:HELLO:", 11)) {
    uint32_t token=0U;
    if (!parseU32Strict(command+11,token) || token==0U) {
      (void)gUsb.writeLineHighPriority("ERR:HOST:SESSION:ARGS");
      return;
    }
    char ack[64];
    std::snprintf(ack,sizeof(ack),"ACK:HOST:SESSION:%lu:%lu",
      static_cast<unsigned long>(token),
      static_cast<unsigned long>(gUsb.transportGeneration()));
    if (gUsb.hostSessionEstablished() && gUsb.hostSessionToken()==token) {
      (void)gUsb.writeLineHighPriority(ack);
      return;
    }
    gUsb.beginHostSession(token);
    if (!gUsb.writeLineHighPriority(ack)) {
      gUsb.invalidateHostSession(); forceRosOffline(); return;
    }
    gUsb.confirmHostSession(); forceRosOffline();
    publishPage(); publishLinkState(); return;
  }
#if defined(BOARD_F103_USBBOOT)
  if (!std::strcmp(command,"BOOT:DFU:ARM")) {
    gDfuArmDeadlineMs=HAL_GetTick()+5000U;
    (void)gUsb.writeLineCritical("ACK:DFU:ARMED",120U);
    return;
  }
  if (!std::strcmp(command,"BOOT:DFU:CONFIRM")) {
    const uint32_t now=HAL_GetTick();
    if (gDfuArmDeadlineMs==0U ||
        static_cast<int32_t>(gDfuArmDeadlineMs-now)<=0) {
      gDfuArmDeadlineMs=0U;
      (void)gUsb.writeLineCritical("ERR:DFU:NOT_ARMED",120U);
      return;
    }
    Bts7960Winch::emergencyStop();
    refreshWinchSafetyInputs();
    if (!firmwareUpdateSafe()) {
      (void)gUsb.writeLineCritical("ERR:DFU:WAIT_SAFE",120U);
      return;
    }
    if (!gUsb.writeLineCritical("ACK:DFU",120U)) return;
    gDfuArmDeadlineMs=0U;
    gUsb.flush(150U); HAL_Delay(20U); enterSystemDfu();
    return;
  }
#endif
  if (!std::strcmp(command,"USB:RECOVER")) {
    forceRosOffline();
    latchAllSafetyStops();
    serviceSafetyControlTx();
    gUsb.requestRecovery();
    return;
  }

  if (!std::strcmp(command,"PING")) {
    (void)gUsb.writeLineCritical("ACK:PONG",120U);
    return;
  }
  if (!std::strcmp(command,"FW:INFO")) {
    (void)gUsb.writeLineCritical("FW:INFO:V3:F103C8:64K",120U);
    return;
  }
  if (!gUsb.hostSessionEstablished()) {
    (void)gUsb.writeLineHighPriority("ERR:HOST:SESSION:REQUIRED");
    return;
  }
#if BTS_WINCH_ENABLED
  if (executeLocalWinchCommand(command)) return;
  if (handleTransactionalWinchCommand(command)) return;
  if (!std::strcmp(command,"WINCH STATUS") || !std::strcmp(command,"STATUS")) {
    char line[192];
    std::snprintf(line,sizeof(line),
      "WINCH:STATE=%s:PWM=%u:APPLIED=%u:TOP=%u:BOTTOM=%u:SW_TICKS=%u:SW_DIR=%d:"
      "T4EN=%u:PA3CFG=%lu:PB8CFG=%lu",
      Bts7960Winch::stateName(),
      static_cast<unsigned>(Bts7960Winch::configuredPwm()),
      static_cast<unsigned>(Bts7960Winch::appliedPwm()),
      Bts7960Winch::topLimitActive()?1U:0U,
      Bts7960Winch::bottomLimitActive()?1U:0U,
      static_cast<unsigned>(Board_BtsDutyTicks()),
      static_cast<int>(Board_BtsDirection()),
      (TIM4->CR1 & TIM_CR1_CEN)?1U:0U,
      static_cast<unsigned long>((GPIOA->CRL >> 12U) & 0xFU),
      static_cast<unsigned long>((GPIOB->CRH >> 0U) & 0xFU));
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS SAMPLE")) {
    uint16_t pb8High=0U, pa3High=0U;
    const uint16_t cntStart=static_cast<uint16_t>(TIM4->CNT);
    uint16_t cntMin=0xFFFFU, cntMax=0U;
    for (uint16_t i=0U; i<400U; ++i) {
      const uint16_t c=static_cast<uint16_t>(TIM4->CNT);
      if (c<cntMin) cntMin=c;
      if (c>cntMax) cntMax=c;
      if ((GPIOB->IDR & GPIO_PIN_8) != 0U) ++pb8High;
      if ((GPIOA->IDR & GPIO_PIN_3) != 0U) ++pa3High;
      Board_DelayUs(20U);
    }
    char line[240];
    std::snprintf(line,sizeof(line),
      "BTS:SAMPLE:PB8_HIGH=%u:PA3_HIGH=%u:MAPR=0x%08lX:RCCR=%lu:LCCR=%lu:"
      "CNT0=%u:CNTMIN=%u:CNTMAX=%u:ARR=%lu:PSC=%lu:CCMR2=0x%08lX:CCER=0x%08lX:"
      "SMCR=0x%08lX:APB1ENR=0x%08lX",
      static_cast<unsigned>(pb8High),
      static_cast<unsigned>(pa3High),
      static_cast<unsigned long>(AFIO->MAPR),
      static_cast<unsigned long>(TIM4->CCR3),
      static_cast<unsigned long>(TIM2->CCR4),
      static_cast<unsigned>(cntStart),
      static_cast<unsigned>(cntMin),
      static_cast<unsigned>(cntMax),
      static_cast<unsigned long>(TIM4->ARR),
      static_cast<unsigned long>(TIM4->PSC),
      static_cast<unsigned long>(TIM4->CCMR2),
      static_cast<unsigned long>(TIM4->CCER),
      static_cast<unsigned long>(TIM4->SMCR),
      static_cast<unsigned long>(RCC->APB1ENR));
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"LIMITS") || !std::strcmp(command,"LS STATUS")) {
    char line[96];
    std::snprintf(line,sizeof(line),
      "LIMITS:READY=%u:TOP=%u:BOTTOM=%u:TOP_RAW=%u:BOTTOM_RAW=%u",
      Bts7960Winch::limitsReady()?1U:0U,
      Bts7960Winch::topLimitActive()?1U:0U,
      Bts7960Winch::bottomLimitActive()?1U:0U,
      Bts7960Winch::topLimitRaw()?1U:0U,
      Bts7960Winch::bottomLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"WINCH HW")) {
    char line[112];
    std::snprintf(line,sizeof(line),
      "WINCH:HW1:MAPR=%08lX:A_CRL=%08lX:B_CRH=%08lX",
      static_cast<unsigned long>(AFIO->MAPR),
      static_cast<unsigned long>(GPIOA->CRL),
      static_cast<unsigned long>(GPIOB->CRH));
    (void)gUsb.writeLineCritical(line,120U);
    std::snprintf(line,sizeof(line),
      "WINCH:HW2:T2CR1=%04lX:T2CCER=%04lX:T2CCR4=%lu:T4CR1=%04lX:T4CCER=%04lX:T4CCR3=%lu",
      static_cast<unsigned long>(TIM2->CR1),
      static_cast<unsigned long>(TIM2->CCER),
      static_cast<unsigned long>(TIM2->CCR4),
      static_cast<unsigned long>(TIM4->CR1),
      static_cast<unsigned long>(TIM4->CCER),
      static_cast<unsigned long>(TIM4->CCR3));
    (void)gUsb.writeLineCritical(line,120U);
    return;
  }
  if (!std::strcmp(command,"BTS MINUP")) {
    refreshWinchSafetyInputs();
    if (!Bts7960Winch::motionAllowed(+1) || Bts7960Winch::topLimitActive()) {
      (void)gUsb.writeLineCritical("ERR:BTS:MINUP:SAFETY",120U); return;
    }
    const bool bottomBefore=Bts7960Winch::bottomLimitRaw();
    Board_BtsSetPwm(1023U,0U);
    const uint32_t deadline=HAL_GetTick()+1500U;
    bool aborted=false;
    bool lifted=false;
    uint32_t topHighSince=0U;
    uint32_t bottomReleasedSince=0U;
    while (static_cast<int32_t>(deadline-HAL_GetTick())>0) {
      Board_RealtimeService();
      const uint32_t now=HAL_GetTick();
      if (Bts7960Winch::topLimitRaw()) {
        if (topHighSince==0U) topHighSince=now;
        if (static_cast<uint32_t>(now-topHighSince)>=10U) {
          aborted=true;
          break;
        }
      } else {
        topHighSince=0U;
      }
      if (bottomBefore && !Bts7960Winch::bottomLimitRaw()) {
        if (bottomReleasedSince==0U) bottomReleasedSince=now;
        if (static_cast<uint32_t>(now-bottomReleasedSince)>=20U) {
          lifted=true;
          break;
        }
      } else {
        bottomReleasedSince=0U;
      }
      __WFI();
    }
    Board_BtsEmergencyCut();
    Bts7960Winch::stop();
    Board_RealtimeDelayMs(40U);
    const bool bottomAfter=Bts7960Winch::bottomLimitRaw();
    char line[112];
    std::snprintf(line,sizeof(line),
      "BTS:MINUP:LIFTED=%u:ABORT=%u:BOTTOM_BEFORE=%u:BOTTOM_AFTER=%u:TOP=%u",
      lifted?1U:0U,aborted?1U:0U,bottomBefore?1U:0U,bottomAfter?1U:0U,
      Bts7960Winch::topLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS LEVELCHECK")) {
    refreshWinchSafetyInputs();
    if (!Bts7960Winch::motionAllowed(+1) || Bts7960Winch::topLimitActive()) {
      (void)gUsb.writeLineCritical("ERR:BTS:LEVELCHECK:SAFETY",120U); return;
    }
    Board_BtsSetPwm(1023U,0U);
    Board_RealtimeDelayMs(25U);
    const bool pb8High=(GPIOB->IDR & GPIO_PIN_8)!=0U;
    const bool pa3Low=(GPIOA->IDR & GPIO_PIN_3)==0U;
    Board_BtsEmergencyCut();

    Board_RealtimeDelayMs(25U);
    Board_BtsSetPwm(0U,1023U);
    Board_RealtimeDelayMs(25U);
    const bool pa3High=(GPIOA->IDR & GPIO_PIN_3)!=0U;
    const bool pb8Low=(GPIOB->IDR & GPIO_PIN_8)==0U;
    Board_BtsEmergencyCut();

    char line[112];
    std::snprintf(line,sizeof(line),
      "BTS:LEVELCHECK:RPWM_HIGH=%u:LPWM_LOW=%u:LPWM_HIGH=%u:RPWM_LOW=%u",
      pb8High?1U:0U,pa3Low?1U:0U,pa3High?1U:0U,pb8Low?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS FULLUP")) {
    refreshWinchSafetyInputs();
    if (!Bts7960Winch::motionAllowed(+1) || Bts7960Winch::topLimitActive()) {
      (void)gUsb.writeLineCritical("ERR:BTS:FULLUP:SAFETY",120U); return;
    }
    const bool bottomBefore=Bts7960Winch::bottomLimitRaw();
    Board_BtsSetPwm(1023U,0U);
    const uint32_t deadline=HAL_GetTick()+350U;
    bool aborted=false;
    uint32_t topHighSince=0U;
    while (static_cast<int32_t>(deadline-HAL_GetTick())>0) {
      Board_RealtimeService();
      const uint32_t now=HAL_GetTick();
      if (Bts7960Winch::topLimitRaw()) {
        if (topHighSince==0U) topHighSince=now;
        if (static_cast<uint32_t>(now-topHighSince)>=10U) {
          aborted=true;
          break;
        }
      } else {
        topHighSince=0U;
      }
      __WFI();
    }
    Board_BtsEmergencyCut();
    Board_RealtimeDelayMs(40U);
    const bool bottomAfter=Bts7960Winch::bottomLimitRaw();
    char line[96];
    std::snprintf(line,sizeof(line),
      "BTS:FULLUP:ABORT=%u:BOTTOM_BEFORE=%u:BOTTOM_AFTER=%u:TOP=%u",
      aborted?1U:0U,bottomBefore?1U:0U,bottomAfter?1U:0U,
      Bts7960Winch::topLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS ESCAPE_TOP_DOWN")) {
    refreshWinchSafetyInputs();
    // This diagnostic is only valid when physically sitting on TOP and moving
    // away from it. Never run it from mid-travel or from the bottom end-stop.
    if (!Bts7960Winch::topLimitRaw() || Bts7960Winch::bottomLimitRaw()) {
      (void)gUsb.writeLineCritical("ERR:BTS:ESCAPE_TOP_DOWN:POSITION",120U); return;
    }
    Bts7960Winch::downHomeLocal();
    if (Bts7960Winch::direction() != -1 || Bts7960Winch::bottomLimitActive()) {
      Bts7960Winch::stop();
      (void)gUsb.writeLineCritical("ERR:BTS:ESCAPE_TOP_DOWN:SAFETY",120U); return;
    }
    Bts7960Winch::stop();
    Board_BtsSetPwm(0U,1023U);
    const uint32_t deadline=HAL_GetTick()+2000U;
    uint32_t topReleasedSince=0U;
    uint32_t bottomActiveSince=0U;
    bool escaped=false, bottomHit=false;
    while (static_cast<int32_t>(deadline-HAL_GetTick())>0) {
      Board_RealtimeService();
      const uint32_t now=HAL_GetTick();
      if (Bts7960Winch::bottomLimitRaw()) {
        if (bottomActiveSince==0U) bottomActiveSince=now;
        if (static_cast<uint32_t>(now-bottomActiveSince)>=2U) { bottomHit=true; break; }
      } else bottomActiveSince=0U;
      if (!Bts7960Winch::topLimitRaw()) {
        if (topReleasedSince==0U) topReleasedSince=now;
        if (static_cast<uint32_t>(now-topReleasedSince)>=20U) { escaped=true; break; }
      } else topReleasedSince=0U;
      __WFI();
    }
    Board_BtsEmergencyCut();
    Bts7960Winch::stop();
    Board_RealtimeDelayMs(40U);
    char line[112];
    std::snprintf(line,sizeof(line),
      "BTS:ESCAPE_TOP_DOWN:ESCAPED=%u:BOTTOM_HIT=%u:TOP=%u:BOTTOM=%u",
      escaped?1U:0U,bottomHit?1U:0U,
      Bts7960Winch::topLimitRaw()?1U:0U,
      Bts7960Winch::bottomLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS FULLREV")) {
    refreshWinchSafetyInputs();
    // Validate through standalone/local DOWN safety (same gate as DOWN HOME),
    // then return the state machine to STOP before the brief board-level test.
    Bts7960Winch::downHomeLocal();
    if (Bts7960Winch::direction() != -1 || Bts7960Winch::bottomLimitActive()) {
      Bts7960Winch::stop();
      (void)gUsb.writeLineCritical("ERR:BTS:FULLREV:SAFETY",120U); return;
    }
    Bts7960Winch::stop();
    const bool bottomBefore=Bts7960Winch::bottomLimitRaw();
    Board_BtsSetPwm(0U,1023U);
    const uint32_t deadline=HAL_GetTick()+350U;
    bool aborted=false;
    uint32_t bottomActiveSince=0U;
    while (static_cast<int32_t>(deadline-HAL_GetTick())>0) {
      Board_RealtimeService();
      const uint32_t now=HAL_GetTick();
      if (Bts7960Winch::bottomLimitRaw()) {
        if (bottomActiveSince==0U) bottomActiveSince=now;
        if (static_cast<uint32_t>(now-bottomActiveSince)>=10U) {
          aborted=true;
          break;
        }
      } else {
        bottomActiveSince=0U;
      }
      __WFI();
    }
    Board_BtsEmergencyCut();
    Board_RealtimeDelayMs(40U);
    const bool bottomAfter=Bts7960Winch::bottomLimitRaw();
    char line[96];
    std::snprintf(line,sizeof(line),
      "BTS:FULLREV:ABORT=%u:BOTTOM_BEFORE=%u:BOTTOM_AFTER=%u:TOP=%u",
      aborted?1U:0U,bottomBefore?1U:0U,bottomAfter?1U:0U,
      Bts7960Winch::topLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS GPIODOWN")) {
    refreshWinchSafetyInputs();
    // Validate through the same local safety path used by standalone DOWN HOME.
    Bts7960Winch::downHomeLocal();
    if (Bts7960Winch::direction() != -1 || Bts7960Winch::bottomLimitActive()) {
      Bts7960Winch::stop();
      (void)gUsb.writeLineCritical("ERR:BTS:GPIODOWN:SAFETY",120U); return;
    }
    Board_BtsEmergencyCut();
    GPIO_InitTypeDef gpioTest{};
    gpioTest.Pin=GPIO_PIN_3;
    gpioTest.Mode=GPIO_MODE_OUTPUT_PP;
    gpioTest.Pull=GPIO_NOPULL;
    gpioTest.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&gpioTest);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
    Board_DelayUs(2000U);
    const bool pa3High=(GPIOA->IDR & GPIO_PIN_3)!=0U;
    const bool pb8Low=(GPIOB->IDR & GPIO_PIN_8)==0U;
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    gpioTest.Mode=GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA,&gpioTest);
    Bts7960Winch::stop();
    char line[96];
    std::snprintf(line,sizeof(line),
      "BTS:GPIODOWN:PA3_HIGH=%u:PB8_LOW=%u:BOTTOM=%u:TOP=%u",
      pa3High?1U:0U,pb8Low?1U:0U,
      Bts7960Winch::bottomLimitRaw()?1U:0U,
      Bts7960Winch::topLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS GPIOUP")) {
    if (!Bts7960Winch::motionAllowed(+1) || Bts7960Winch::topLimitActive()) {
      (void)gUsb.writeLineCritical("ERR:BTS:GPIOUP:SAFETY",120U); return;
    }
    const bool bottomBefore=Bts7960Winch::bottomLimitRaw();
    Board_BtsSetPwm(0U,0U);
    GPIO_InitTypeDef gpioTest{};
    gpioTest.Pin=GPIO_PIN_8;
    gpioTest.Mode=GPIO_MODE_OUTPUT_PP;
    gpioTest.Pull=GPIO_NOPULL;
    gpioTest.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&gpioTest);
    bool aborted=false;
    for (uint16_t i=0U; i<250U; ++i) {
      if (!Bts7960Winch::motionAllowed(+1) || Bts7960Winch::topLimitActive()) {
        aborted=true; break;
      }
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
      Board_RealtimeDelayMs(1U);
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
      Board_RealtimeDelayMs(1U);
    }
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
    gpioTest.Mode=GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB,&gpioTest);
    Board_BtsSetPwm(0U,0U);
    Board_RealtimeDelayMs(40U);
    const bool bottomAfter=Bts7960Winch::bottomLimitRaw();
    char line[96];
    std::snprintf(line,sizeof(line),
      "BTS:GPIOUP:ABORT=%u:BOTTOM_BEFORE=%u:BOTTOM_AFTER=%u:TOP=%u",
      aborted?1U:0U,bottomBefore?1U:0U,bottomAfter?1U:0U,
      Bts7960Winch::topLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"BTS PINTEST")) {
    // Diagnostic only: actuator must be idle. Drive PB8 HIGH for 2 ms,
    // sample the actual pin level, then restore the timer-DMA GPIO mode.
    if (Bts7960Winch::direction() != 0 || Bts7960Winch::motionPending()) {
      (void)gUsb.writeLineCritical("ERR:BTS:PINTEST:BUSY",120U); return;
    }
    Board_BtsSetPwm(0U,0U);
    GPIO_InitTypeDef testGpio{};
    testGpio.Pin=GPIO_PIN_8;
    testGpio.Mode=GPIO_MODE_OUTPUT_PP;
    testGpio.Pull=GPIO_NOPULL;
    testGpio.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&testGpio);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
    Board_DelayUs(2000U);
    const bool high=(GPIOB->IDR & GPIO_PIN_8)!=0U;
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
    // Production BTS backend uses timer-DMA with PB8 as GPIO push-pull.
    testGpio.Mode=GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB,&testGpio);
    char line[64];
    std::snprintf(line,sizeof(line),"BTS:PINTEST:PB8=%u",high?1U:0U);
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"WINCH FAULT CLEAR")) {
    refreshWinchSafetyInputs();
    (void)gUsb.writeLineCritical(
      Bts7960Winch::clearFault()?"ACK:WINCH:FAULT_CLEAR":"ERR:WINCH:FAULT_CLEAR",120U);
    markUiDirty(); return;
  }
  if (!std::strncmp(command,"WINCH PWM ",10)) {
    uint32_t pwm=0U;
    const bool ok=parseU32Strict(command+10,pwm) &&
      pwm<=Bts7960Winch::PWM_MAX &&
      Bts7960Winch::setPwm(static_cast<uint16_t>(pwm),true);
    (void)gUsb.writeLineCritical(ok?"ACK:WINCH:PWM":"ERR:WINCH:PWM",120U);
    markUiDirty(); return;
  }
  if (executeWinchCommand(command,0U,false)) return;
#endif

  if (!std::strcmp(command,"GET:STATE")) {
    publishPage(); publishLinkState(); return;
  }
  if (!std::strcmp(command,"USB:STATUS")) {
    char line[112];
    std::snprintf(line,sizeof(line),"USB:STAT:host=%u,gen=%lu,rx=%lu,drop=%lu",
      gUsb.hostSessionEstablished()?1U:0U,
      static_cast<unsigned long>(gUsb.transportGeneration()),
      static_cast<unsigned long>(gUsb.rxPacketCount()),
      static_cast<unsigned long>(gUsb.rxDropped()));
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (!std::strcmp(command,"TOUCH:STATUS")) {
    char line[180];
    std::snprintf(line,sizeof(line),
      "TOUCH:STATUS:READ=%lu:PRESS_REJECT=%lu:BOUNDS_REJECT=%lu:Z=%u:RAW=%u,%u:XY=%u,%u:MODE=%s:PAGE=%s",
      static_cast<unsigned long>(gDiagnostics.touchReadCount),
      static_cast<unsigned long>(gDiagnostics.touchRejectFastCount),
      static_cast<unsigned long>(tft.touchRejectBoundsCount()),
      static_cast<unsigned>(tft.touchCurrentZ()),
      static_cast<unsigned>(tft.touchLastRawX()),
      static_cast<unsigned>(tft.touchLastRawY()),
      static_cast<unsigned>(tft.touchLastX()),
      static_cast<unsigned>(tft.touchLastY()),
      HmiTouchService::modeName(),gOperatorUi.wireName());
    (void)gUsb.writeLineCritical(line,120U); return;
  }
  if (HmiTouchService::handleCommand(command)) {
    Bts7960Winch::emergencyStop();
    if (HmiTouchService::active()) triggerLocalEmergencyStop();
    gOperatorUi.cancelTouch(); return;
  }
  if (!std::strcmp(command,"HMI REDRAW") ||
      !std::strcmp(command,"HMI RESET") ||
      !std::strcmp(command,"TFT RESET")) {
    Bts7960Winch::emergencyStop(); HmiTouchService::cancel();
    gOperatorUi.cancelTouch(); beginDisplayInitialization();
    (void)gUsb.writeLineCritical("ACK:HMI:RECOVERY",120U); return;
  }
  if (!std::strncmp(command,"ROS:",4)) {
    if (command[4]=='1') markRosHeartbeat(); else forceRosOffline();
    return;
  }
  if (!std::strncmp(command,"SYS:",4)) {
    const char *v=command+4;
    gTelemetry.systemStatus=!std::strcmp(v,"FAULT")?SYS_FAULT:
      (!std::strcmp(v,"READY")?SYS_READY:SYS_NOT_READY);
    markUiDirty(); return;
  }
  if (!std::strncmp(command,"STATE:",6)) {
    const char *v=command+6;
    gTelemetry.state=!std::strcmp(v,"RUNNING")?STATE_RUNNING:
      (!std::strcmp(v,"FAULT")?STATE_FAULT:
       (!std::strcmp(v,"STANDBY")?STATE_STANDBY:STATE_STOPPED));
    markUiDirty(); return;
  }
  if (!std::strncmp(command,"NAV:",4)) {
    const char *v=command+4;
    gTelemetry.navigationStatus=
      (!std::strcmp(v,"NAVIGATING") || !std::strcmp(v,"ACTIVE") ||
       !std::strcmp(v,"QUEUED")) ? NAV_NAVIGATING : NAV_IDLE;
    return;
  }
  if (!std::strcmp(command,"ESTOP:RESET")) {
    (void)gUsb.writeLineHighPriority(
      resetLocalEmergencyStop()?"ACK:ESTOP:RESET":"ERR:ESTOP:RESET:UNSAFE");
    return;
  }
  if (!std::strncmp(command,"ESTOP:",6)) {
    if (command[6]=='0' && command[7]=='\0') setHostEmergencyStop(false);
    else if (command[6]=='1' && command[7]=='\0') setHostEmergencyStop(true);
    else (void)gUsb.writeLineHighPriority("ERR:ESTOP:ARGS");
    return;
  }
  // Rich ESC/perception/navigation display telemetry is intentionally ignored.
}
