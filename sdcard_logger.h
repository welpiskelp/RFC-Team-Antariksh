/*************************************************************************
   COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Rik Seth

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission
   from Team Antariksh.

*************************************************************************/
#ifndef SDCARD_LOGGER_H
#define SDCARD_LOGGER_H

#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

extern logger_interface_t sdcard_logger_interface;
void sdcard_force_sync(void);
void sdcard_flush_from_flash(void)
/* default log filename (change if you want) */
#define SDCARD_LOG_FILENAME "LOGNEW.TXT"

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_LOGGER_H */
