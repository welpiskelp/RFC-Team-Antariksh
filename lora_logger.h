/*************************************************************************
COPYRIGHT NOTICE
   (c) 2025 Team Antariksh
   Author: Aarush Jaiswal
   All rights reserved.
*************************************************************************/

#ifndef LORA_LOGGER_H
#define LORA_LOGGER_H

#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* Provide a pre-declared logger_interface_t */
    extern logger_interface_t lora_logger_interface;

    /* Helper function to send an ACK during telecommand phase */
    void lora_send_ack(const char* ack_msg);

#ifdef __cplusplus
}
#endif

#endif /* LORA_LOGGER_H */