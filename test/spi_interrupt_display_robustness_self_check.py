#!/usr/bin/env python3
from pathlib import Path
import re, sys
ROOT=Path(__file__).resolve().parents[1]

def rd(p): return (ROOT/p).read_text(errors='replace')
def need(c,msg):
    if not c:
        print('FAIL',msg); sys.exit(1)
    print('PASS',msg)

h=rd('include/HmiDisplay.h'); cpp=rd('src/HmiDisplay.cpp'); main=rd('src/main.cpp')
ui=rd('src/UiShell.h'); board=rd('src/BoardSupport.cpp'); usb=rd('src/usb/usbd_conf.c'); diag=rd('src/Diagnostics.h')

# SPI ownership/recovery
for tok in ['SpiOwner::IDLE','SpiOwner::TFT_WRITE','SpiOwner::TFT_READ','SpiOwner::TOUCH','spi_bus_conflict_count_']:
    need(tok in cpp or tok in h, f'explicit shared-SPI ownership: {tok}')
need('kSpiWaitTimeoutUs' in h and 'DWT->CYCCNT' in cpp and 'spi_timeout_count_' in cpp,
     'SPI BSY wait is DWT-bounded and counted')
need('Board_ReinitSpi1' in cpp and 'recoverSpi()' in cpp, 'SPI recovery path reinitializes peripheral')
need('TftCs(true);\n  TouchCs(true);' in cpp, 'recovery/transaction preparation deasserts both chip selects')
need('setWindowTx' in cpp and 'writeColorTx' in cpp and 'pushImageTx' in cpp,
     'window + pixel payload use batched transaction primitives')
need('HAL_SPI_Transmit(' in cpp and 'kSpiHalTimeoutMs' in cpp, 'HAL transfers have bounded timeout')

# IRQ hierarchy
need(re.search(r'SetPriority\(USART1_IRQn,\s*0', board) is not None, 'USART1/VESC priority 0')
need(re.search(r'SetPriority\(USART2_IRQn,\s*1', board) is not None, 'USART2/GNSS priority 1')
need(re.search(r'SetPriority\(OTG_FS_IRQn,\s*2', usb) is not None, 'USB OTG FS priority 2')
need(re.search(r'SetPriority\(TIM1_TRG_COM_TIM11_IRQn,\s*3', board) is not None, 'TIM11 watchdog priority 3')
for irqbody in re.findall(r'extern\s+"C"\s+void\s+\w+_IRQHandler\([^)]*\)\s*\{([^}]*)\}', board+usb, re.S):
    need('HAL_SPI_' not in irqbody and 'draw' not in irqbody, 'ISR body does not render or access SPI')

# Dirty renderer / cache
content_start=ui.find('inline void drawUiContent(')
content_end=ui.find('inline void drawUiFrame(', content_start)
content=ui[content_start:content_end] if content_start >= 0 and content_end > content_start else ''
need(content and 'CONTENT_Y - 1' not in content and 'fillRect(0, CONTENT' not in content, 'incremental content path has no content-wide clear')
need('UiDirtyBits' in ui and 'uiDirtyMask' in main, 'fixed-size dirty mask controls refresh')
need('UiMetricCacheEntry' in ui and 'strncmp(cache->value' in ui, 'dynamic metric values are cached')
need('gUiDynamicPass' in ui and 'drawUiTextPadded' in ui, 'dynamic pass repaints bounded value boxes')
need('text_tile_[320U * kTextTileRows]' in h and 'pushImage(ax0,stripY,aw,sh,text_tile_)' in cpp,
     'font renderer uses fixed RGB565 tile instead of per-pixel SPI writes')

# Touch fast reject
need('touch_reject_fast_count_' in cpp and 'if(z<=threshold)' in cpp, 'XPT2046 pressure fast-reject implemented')
need('Median3' in cpp and 'xs[3]' in cpp and 'ys[3]' in cpp, 'pressed touch uses 3-sample median filter')
need('validTouch(' not in cpp, 'legacy five-pass touch validation removed')

# Diagnostics / degraded mode
for f in ['spiTransactions','spiBytesTx','spiTimeoutCount','spiHalErrorCount','spiRecoveryCount',
          'spiBusConflictCount','touchReadCount','touchRejectFastCount','displayBytesLastFrame',
          'displayBytesMaxFrame','uiDrawLastMs','uiDrawMaxMs','maxServiceGapMs','displayReady','displayFaulted']:
    need(f in diag and f in main, f'diagnostics wired: {f}')
need('serviceDisplayRecovery' in main and 'displayFaulted()' in main, 'display fault degrades and retries independently')
need('TFT:DIAG' in main, 'runtime SPI/frame diagnostics command available')
print('PASS F4GATEWAY_SPI_INTERRUPT_DISPLAY_ROBUSTNESS')
