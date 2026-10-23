// Device and string descriptors. VID/PID, strings and the serial number
// derivation match the old STM32_USB_Device_VCP-1.2.0 usbd_desc.c, so hosts
// keep seeing the same device.

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#ifndef USBD_VID
#define USBD_VID 0x0483
#endif
#ifndef USBD_PID
#define USBD_PID 0x5740
#endif
#define USBD_LANGID_STRING 0x409
#ifndef USBD_MANUFACTURER_STRING
#define USBD_MANUFACTURER_STRING "STMicroelectronics"
#endif
#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING "STM32 Virtual ComPort"
#endif
#define USBD_CONFIGURATION_STRING "VCP Config"
#define USBD_INTERFACE_STRING "VCP Interface"

#define DEVICE_ID ((__IO uint32_t *)0x1FFF7A10)

static uint8_t *dev_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *langid_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *manufacturer_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *product_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *serial_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *config_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *interface_desc(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef VCP_Desc = {
    dev_desc,
    langid_desc,
    manufacturer_desc,
    product_desc,
    serial_desc,
    config_desc,
    interface_desc,
};

__ALIGN_BEGIN static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12,                 /* bLength */
    USB_DESC_TYPE_DEVICE, /* bDescriptorType */
    0x00,                 /* bcdUSB */
    0x02,
    0x00,             /* bDeviceClass */
    0x00,             /* bDeviceSubClass */
    0x00,             /* bDeviceProtocol */
    USB_MAX_EP0_SIZE, /* bMaxPacketSize */
    LOBYTE(USBD_VID), /* idVendor */
    HIBYTE(USBD_VID),
    LOBYTE(USBD_PID), /* idProduct */
    HIBYTE(USBD_PID),
    0x00, /* bcdDevice rel. 2.00 */
    0x02,
    USBD_IDX_MFC_STR,          /* Index of manufacturer string */
    USBD_IDX_PRODUCT_STR,      /* Index of product string */
    USBD_IDX_SERIAL_STR,       /* Index of serial number string */
    USBD_MAX_NUM_CONFIGURATION /* bNumConfigurations */
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING),
};

__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

__ALIGN_BEGIN static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static void int_to_unicode(uint32_t value, uint8_t *pbuf, uint8_t len) {
  for(uint8_t idx = 0; idx < len; idx++) {
    if((value >> 28) < 0xA) {
      pbuf[2 * idx] = (value >> 28) + '0';
    } else {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }
    value              = value << 4;
    pbuf[2 * idx + 1] = 0;
  }
}

static uint8_t *dev_desc(USBD_SpeedTypeDef speed, uint16_t *length) {
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

static uint8_t *langid_desc(USBD_SpeedTypeDef speed, uint16_t *length) {
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

static uint8_t *manufacturer_desc(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *product_desc(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *serial_desc(USBD_SpeedTypeDef speed, uint16_t *length) {
  uint32_t deviceserial = DEVICE_ID[0] + DEVICE_ID[2];
  if(deviceserial != 0) {
    int_to_unicode(deviceserial, &USBD_StringSerial[2], 8);
    int_to_unicode(DEVICE_ID[1], &USBD_StringSerial[18], 4);
  }
  *length = USB_SIZ_STRING_SERIAL;
  return USBD_StringSerial;
}

static uint8_t *config_desc(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *interface_desc(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}
