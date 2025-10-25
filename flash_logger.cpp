/*************************************************************************
   COPYRIGHT NOTICE
   (c) 2025 Team Antariksh
   Author: Rik Seth & Aarush Jaiswal

   Flash logger – buffered EEPROM logger.
   Logs text entries sequentially into EEPROM (wraps around).
   Does not interfere with state_recovery section (0x00–0x3F).
*************************************************************************/

#include "flash_logger.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SD.h>

static uint16_t write_ptr = EEPROM_LOG_START;   // Next write address
static bool flash_ready = false;

/* ---------- internal helpers ---------- */
static void advance_pointer() {
    write_ptr++;
    if (write_ptr > EEPROM_LOG_END)
        write_ptr = EEPROM_LOG_START; // wrap around
}

/* ---------- interface init ---------- */
static void flashlogger_init(void) {
    EEPROM.begin(EEPROM_LOG_SIZE);
    flash_ready = true;
}

/* ---------- interface write ---------- */
static void flashlogger_write(const char* msg, size_t len) {
    if (!flash_ready || len == 0) return;

    for (size_t i = 0; i < len; i++) {
        EEPROM.write(write_ptr, (uint8_t)msg[i]);
        advance_pointer();
    }

    // Ensure newline at end
    EEPROM.write(write_ptr, '\n');
    advance_pointer();

    EEPROM.commit(); // Commit changes to flash
}

/* ---------- EEPROM → SD flush ---------- */
void flashlogger_flush_to_sd(const char* filename) {
    if (!flash_ready) return;

    File dumpFile = SD.open(filename, FILE_WRITE);
    if (!dumpFile) {
        Serial.println("FlashLogger: SD open failed");
        return;
    }

    Serial.println("FlashLogger: dumping EEPROM logs to SD...");

    for (uint16_t addr = EEPROM_LOG_START; addr <= EEPROM_LOG_END; addr++) {
        uint8_t b = EEPROM.read(addr);
        dumpFile.write(b);
    }

    dumpFile.flush();
    dumpFile.close();
    Serial.println("FlashLogger: dump complete");
}

/* ---------- exported logger interface ---------- */
logger_interface_t flash_logger_interface = {
    .init = flashlogger_init,
    .write = flashlogger_write,
    .is_immediate_flush = false // buffered
};
