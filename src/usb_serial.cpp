/*
 * USB CDC virtual serial port — OTG_FS on PA11 (DM) / PA12 (DP).
 * Appears as /dev/ttyACMx on Linux, COM port on Windows.
 *
 * Adapted from ChibiOS testhal/STM32/multi/USB_CDC.
 * VID 0x0483 (ST), PID 0x5740 (Virtual COM Port).
 */

#include "ch.h"
#include "hal.h"
#include "src/usb_serial.hpp"

SerialUSBDriver SDU1;

#define USB_DATA_EP      1u
#define USB_INTR_EP      2u

/* ── USB descriptors ──────────────────────────────────────────────────────── */

static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE(0x0200,       /* bcdUSB 2.0                          */
                  0x02,         /* bDeviceClass (CDC)                  */
                  0x00,         /* bDeviceSubClass                     */
                  0x00,         /* bDeviceProtocol                     */
                  0x40,         /* bMaxPacketSize0 (64)                */
                  0x0483,       /* idVendor (ST)                       */
                  0x5740,       /* idProduct (Virtual COM)             */
                  0x0200,       /* bcdDevice                           */
                  1,            /* iManufacturer                       */
                  2,            /* iProduct                            */
                  3,            /* iSerialNumber                       */
                  1)            /* bNumConfigurations                  */
};

static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

static const uint8_t vcom_configuration_descriptor_data[67] = {
  USB_DESC_CONFIGURATION(67, 0x02, 0x01, 0, 0xC0, 50),
  /* Interface 0: CDC control */
  USB_DESC_INTERFACE(0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0),
  /* Header Functional Descriptor */
  USB_DESC_BYTE(5),  USB_DESC_BYTE(0x24), USB_DESC_BYTE(0x00), USB_DESC_BCD(0x0110),
  /* Call Management Functional Descriptor */
  USB_DESC_BYTE(5),  USB_DESC_BYTE(0x24), USB_DESC_BYTE(0x01),
  USB_DESC_BYTE(0x00), USB_DESC_BYTE(0x01),
  /* ACM Functional Descriptor */
  USB_DESC_BYTE(4),  USB_DESC_BYTE(0x24), USB_DESC_BYTE(0x02), USB_DESC_BYTE(0x02),
  /* Union Functional Descriptor */
  USB_DESC_BYTE(5),  USB_DESC_BYTE(0x24), USB_DESC_BYTE(0x06),
  USB_DESC_BYTE(0x00), USB_DESC_BYTE(0x01),
  /* EP2 IN (interrupt, 16 bytes, 255 ms interval) */
  USB_DESC_ENDPOINT(USB_INTR_EP | 0x80, 0x03, 0x0010, 0xFF),
  /* Interface 1: CDC data */
  USB_DESC_INTERFACE(0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00),
  /* EP1 OUT (bulk, 64 bytes) */
  USB_DESC_ENDPOINT(USB_DATA_EP,         0x02, 0x0040, 0x00),
  /* EP1 IN  (bulk, 64 bytes) */
  USB_DESC_ENDPOINT(USB_DATA_EP | 0x80,  0x02, 0x0040, 0x00)
};

static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/* String 0: language ID (U.S. English) */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  USB_DESC_WORD(0x0409)
};

/* String 1: manufacturer */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(14),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  'B', 0, 'P', 0, 'R', 0, 'L', 0, '-', 0, 'F', 0
};

/* String 2: product */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(32),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  'B', 0, 'P', 0, 'R', 0, 'L', 0, ' ', 0,
  'D', 0, 'e', 0, 'b', 0, 'u', 0, 'g', 0, ' ', 0,
  'U', 0, 'S', 0, 'B', 0
};

/* String 3: serial number */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang)
{
  (void)usbp; (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:       return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION: return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4u) return &vcom_strings[dindex];
    break;
  }
  return nullptr;
}

/* ── Endpoint state objects ───────────────────────────────────────────────── */

static USBInEndpointState  ep1instate;
static USBOutEndpointState ep1outstate;

static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK,
  nullptr,
  sduDataTransmitted,
  sduDataReceived,
  0x0040,
  0x0040,
  &ep1instate,
  &ep1outstate,
  2,
  nullptr
};

static USBInEndpointState ep2instate;

static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  nullptr,
  sduInterruptTransmitted,
  nullptr,
  0x0010,
  0x0000,
  &ep2instate,
  nullptr,
  1,
  nullptr
};

/* ── USB event callbacks ──────────────────────────────────────────────────── */

static void usb_event(USBDriver *usbp, usbevent_t event)
{
  switch (event) {
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();
    usbInitEndpointI(usbp, USB_DATA_EP,  &ep1config);
    usbInitEndpointI(usbp, USB_INTR_EP,  &ep2config);
    sduConfigureHookI(&SDU1);
    chSysUnlockFromISR();
    return;
  case USB_EVENT_RESET:
  case USB_EVENT_UNCONFIGURED:
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();
    sduSuspendHookI(&SDU1);
    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    chSysLockFromISR();
    sduWakeupHookI(&SDU1);
    /* If device was already configured before suspend, restore SDU_READY.
     * Linux autosuspend causes SUSPEND→WAKEUP without a new SET_CONFIGURATION,
     * so USB_EVENT_CONFIGURED never re-fires and SDU stays stuck in SDU_STOP. */
    if (usbp->state == USB_ACTIVE)
        sduConfigureHookI(&SDU1);
    chSysUnlockFromISR();
    return;
  default:
    return;
  }
}

static void sof_handler(USBDriver *usbp)
{
  (void)usbp;
  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  osalSysUnlockFromISR();
}

static const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  sduRequestsHook,
  sof_handler
};

static const SerialUSBConfig serusbcfg = {
  &USBD1,
  USB_DATA_EP,
  USB_DATA_EP,
  USB_INTR_EP
};

/* ── Public init ──────────────────────────────────────────────────────────── */

void usb_serial_init(void)
{
  /* The bootloader leaves OTG_FS running with D+ asserted (DCTL_SDIS=0).
   * If we call usbStart while the USB host is actively sending SOF frames,
   * the OTG interrupt fires continuously inside otg_core_reset, keeping
   * GRSTCTL_AHBIDL clear, and the soft-reset loop hangs forever.
   *
   * Fix (mirrors ArduPilot): disconnect D+ first (host stops sending),
   * then start, then reconnect so the host enumerates fresh. */
  rccEnableOTG_FS(true);          /* ensure AHB1 clock on — bootloader may have
                                     disabled it, though it rarely does        */
  usbDisconnectBus(&USBD1);       /* set DCTL_SDIS → host sees disconnect      */
  chThdSleepMilliseconds(5);      /* give host time to register the disconnect  */

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbStart(&USBD1, &usbcfg);     /* safe: no host traffic; AHBIDL loop exits   */
  usbConnectBus(&USBD1);          /* clear DCTL_SDIS → host enumerates device   */
}
