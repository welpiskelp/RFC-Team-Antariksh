/*************************************************************************
   COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Aarush Jaiswal

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission 
   from Team Antariksh.

*************************************************************************/
#include "sdcard_logger.h"
#include <SD.h>
#include <Arduino.h>
#define EEPROM_LOG_START 64
#define EEPROM_LOG_END 1024
/*
  sdcard_logger using Arduino SD.h and BUILTIN_SDCARD (Teensy).
  - Non-immediate by default (relies on logger_flush()).
  - Keeps the File open after init to reduce open/close overhead.
  - Uses File.write() to write raw bytes. Calls flush() on logger_flush()
    indirectly if you call SD.sync() or close; here we expose a small
    helper to sync on demand via a public function (optional).
*/

static File logFile;
static bool sd_ready = false;

/* init: initialize SD and open file for append */
static void sdcard_init(void) {
    sd_ready = false;

    // Attempt to initialize the builtin SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD: begin failed!");
        // failed to initialize SD
        return;
    }
    Serial.println("SD: begun OK");
    // Open the log file for append (create if not exists)
    // SD.open returns a File object
    logFile = SD.open(SDCARD_LOG_FILENAME, FILE_WRITE);
    if (!logFile) {
        Serial.println("SD: open failed!");
        File f = SD.open(SDCARD_LOG_FILENAME, FILE_WRITE);
        if (!f) {
            // couldn't create/open file
            return;
        }
        f.close();
        logFile = SD.open(SDCARD_LOG_FILENAME, FILE_WRITE);
        if (!logFile) {
            return;
        }
    }

    // Move to end to append
    logFile.seek(logFile.size());
    sd_ready = true;
}

/* write: append msg bytes to the open file (does not call sync/flush each time) */
static void sdcard_write(const char *msg, size_t len) {
    if (!sd_ready) return;
    if (len == 0) return;

    // Write the bytes (File::write accepts const uint8_t*, size)
    size_t wrote = logFile.write((const uint8_t*)msg, len);
    (void)wrote;

    // Ensure newline at end for readability
    if (len == 0 || msg[len - 1] != '\n') {
        logFile.write((const uint8_t *)"\n", 1);
    }
    logFile.flush();

    // Do NOT call logFile.flush() on every write to reduce wear & latency.
    // Call logger_flush() to force persistence which will call the
    // sdcard logger's write (and you can add a manual sync if desired).
}

/* optional helper: force a flush to SD (call from logger_flush or elsewhere) */
//static void sdcard_force_sync(void)
static void sdcard_force_sync(void) {
    if (!sd_ready) return;
    // File::flush() is available in many Arduino SD implementations.
    // If flush() is not available for your SD library version, you can
    // close and reopen the file instead (costly).
    #if defined(__AVR__) || defined(ARDUINO)
    // many cores provide flush()
    logFile.flush();
    #else
    // attempt flush; if not available, close & reopen as fallback
    #ifdef FILE_FLUSHABLE
    logFile.flush();
    #else
    // fallback: close and reopen (expensive)
    logFile.close();
    logFile = SD.open(SDCARD_LOG_FILENAME, FILE_WRITE);
    logFile.seek(logFile.size());
    #endif
    #endif
}


void sdcard_flush_from_flash() {
   
}

/* Exported logger_interface_t object */
logger_interface_t sdcard_logger_interface = {
    .init = sdcard_init,
    .write = sdcard_write,
    .is_immediate_flush = false /* non-immediate: buffered by logger.c */
};

/* If you want logger_flush() to also call sdcard_force_sync() to persist to disk,
   modify logger_flush() in your logger.cpp to call sdcard_force_sync() after calling
   sdcard_logger_interface.write(...) for each entry, or expose a function here to
   be called from logger_flush(). */
