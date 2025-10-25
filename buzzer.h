/*************************************************************************
   COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Rik Seth

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission 
   from Team Antariksh.
*************************************************************************/

#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

/* buzzer pin definition */
#define BUZZER_PIN 23 // Changed to pin 23

/* buzzer beep codes */
typedef enum {
    BUZZER_CODE_INIT_SUCCESS = 0,   /* 1 long beep: successful initialization (pre-launch) */
    BUZZER_CODE_INIT_FAIL    = 1,   /* 3 short beeps: initialization failure (pre-launch) */
    BUZZER_CODE_GENERAL_ERR  = 2,   /* 4 short beeps: general/system error (pre-launch/ground test) */
    BUZZER_CODE_LANDING      = 3,   /* 2 long beeps: touchdown confirmation (post-flight/ground recovery) */
    BUZZER_CODE_APOGEE =4,
} buzzer_code_t;

/* Initialize the buzzer pin */
void buzzer_init();

/* play a beep code sequence */
void buzzer_beep_code(buzzer_code_t code);

#endif /* BUZZER_H */