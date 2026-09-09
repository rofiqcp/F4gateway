#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool boot_usb_begin(void);
void boot_usb_end(void);
bool boot_usb_connected(void);
int boot_usb_available(void);
int boot_usb_read(void);
bool boot_usb_write_line(const char *line, uint32_t timeout_ms);
void boot_usb_flush(uint32_t timeout_ms);
void boot_usb_on_receive(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif
