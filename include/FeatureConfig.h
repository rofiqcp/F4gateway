#pragma once

#ifndef MCP2515_ENABLED
#define MCP2515_ENABLED 1
#endif

#ifndef BTS_WINCH_ENABLED
#define BTS_WINCH_ENABLED 1
#endif

#ifndef HMI_F4_LAYOUT
#define HMI_F4_LAYOUT 1
#endif

#if MCP2515_ENABLED && !defined(NEO3PRO)
#error "MCP2515_ENABLED=1 requires the NEO3PRO profile"
#endif

#if BTS_WINCH_ENABLED && defined(NEO3)
#error "BTS winch uses PA2/PB8; these pins conflict with the legacy NEO3 UART/I2C profile"
#endif
