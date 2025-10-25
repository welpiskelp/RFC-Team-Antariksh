/*************************************************************************
COPYRIGHT NOTICE
   (c) 2025 Team Antariksh
   Author: Aarush Jaiswal & Rik Seth

   Flash (EEPROM) logger module.
   - Uses EEPROM as non-volatile ring buffer storage.
   - Designed to work seamlessly with logger.h/logger.cpp.
   - Data can be dumped later to SD card for post-flight recovery.
*************************************************************************/

#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* Exported interface for registration with logger_register_interface() */
    extern logger_interface_t flash_logger_interface;

    /* EEPROM region reserved for logging (avoid state_recovery area) */
#define FLASH_LOG_START_ADDR   64
#define FLASH_LOG_END_ADDR     1023
#define FLASH_LOG_TOTAL_BYTES  (FLASH_LOG_END_ADDR - FLASH_LOG_START_ADDR + 1)

    /* Derived limits (match logger.h’s log entry size) */
#define FLASH_LOG_ENTRY_SIZE   LOG_ENTRY_SIZE
#define FLASH_LOG_ENTRIES      (FLASH_LOG_TOTAL_BYTES / FLASH_LOG_ENTRY_SIZE)

    /**
     * @brief Dump all EEPROM-stored log entries to SD card.
     * @param filename File to write logs into (e.g. "FLASH_DUMP.TXT")
     */
    void flashlogger_flush_to_sd(const char* filename);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_LOGGER_H */
