/*************************************************************************
COPYRIGHT NOTICE
   (c) 2025 Team Antariksh
   Author: Rik Seth & Aarush Jaiswal

   Flash logger for intermediate EEPROM/flash logging.
   Logs telemetry strings to EEPROM sequentially, with wrap-around.
   Data is later dumped to SD card via flashlogger_flush_to_sd().
*************************************************************************/

#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

    extern logger_interface_t flash_logger_interface;

    /* EEPROM address region reserved for logs */
#define EEPROM_LOG_START   64
#define EEPROM_LOG_END     1023
#define EEPROM_LOG_SIZE    (EEPROM_LOG_END - EEPROM_LOG_START + 1)

    /**
     * @brief Push all EEPROM-stored logs to SD card.
     * @param filename  File name to write to (e.g. "EE_DUMP.TXT")
     */
    void flashlogger_flush_to_sd(const char* filename);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_LOGGER_H */
