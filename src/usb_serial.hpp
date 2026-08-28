#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "hal.h"

/*
 * USB CDC virtual serial port (micro USB, OTG_FS).
 * Init: call usb_serial_init() once after halInit()/chSysInit().
 * Use:  cast SDU1 to BaseSequentialStream* for chprintf() etc.
 *
 * Allow ~1500 ms after usb_serial_init() before writing, for host
 * USB enumeration to complete.
 */
extern SerialUSBDriver SDU1;

void usb_serial_init(void);

#ifdef __cplusplus
}
#endif
