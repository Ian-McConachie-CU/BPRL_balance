#pragma once

// Starts the USB command-line thread and the safety watchdog thread.
// Call once after can_drv_init(), rmd_init(), gim_init(), usb_serial_init().
void cmd_shell_start(void);
