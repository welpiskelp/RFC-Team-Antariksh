/*************************************************************************
   COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Rik Seth

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission
   from Team Antariksh.

*************************************************************************/
#include "com_logger.h"
#include <Arduino.h>

/* Serial logger using USB Serial (Serial) directly.
   On Teensy 4.x, Serial is usb_serial_class, not HardwareSerial.
   Immediate flush is true.
*/
static const uint32_t COM_BAUD = 115200;

/* init function */
static void com_init(void) {
    Serial.begin(COM_BAUD);
    // small delay to allow USB enumeration
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 1000) { }
}

/* write function */
static void com_write(const char *msg, size_t len) {
    if (len > 0) {
        Serial.write((const uint8_t*)msg, len);
    }
    // ensure newline for readability
    if (len == 0 || msg[len - 1] != '\n') {
        Serial.write('\n');
    }
}

/* Exported interface object */
logger_interface_t com_logger_interface = {
    .init = com_init,
    .write = com_write,
    .is_immediate_flush = true
};
