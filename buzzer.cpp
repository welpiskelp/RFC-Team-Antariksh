/*************************************************************************
COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Rik Seth

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission
   from Team Antariksh.
*************************************************************************/

#include "buzzer.h"

// Define a standard frequency for the beeps (A5 note)
#define BEEP_FREQUENCY 880

void buzzer_init() {
    // pinMode is handled by tone() but it's good practice to set it.
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Ensure pin is off initially
}

// This function is now updated to use tone() for a passive buzzer
static void beep(int count, int duration = 200, int pause = 100) {
    for (int i = 0; i < count; ++i) {
        tone(BUZZER_PIN, BEEP_FREQUENCY); // Play the tone
        delay(duration);                  // Let it play for the specified duration
        noTone(BUZZER_PIN);               // Stop the tone
        delay(pause);                     // Wait for the pause duration
    }
}

void buzzer_beep_code(buzzer_code_t code) {
    switch (code) {
        case BUZZER_CODE_INIT_SUCCESS:
            /* One long beep (pre-launch success) */
            beep(1, 500, 200);
            break;
        case BUZZER_CODE_INIT_FAIL:
            /* Three short beeps (pre-launch failure) */
            beep(3, 150, 150);
            break;
        case BUZZER_CODE_GENERAL_ERR:
            /* Four short beeps (system/general error) */
            beep(4, 100, 100);
            break;
        case BUZZER_CODE_LANDING:
            /* Two long beeps (touchdown/ground recovery) */
            beep(2, 400, 200);
            break;
        case BUZZER_CODE_APOGEE:
            beep(2, 400, 200);
            break;
        default:
            /* Unknown code, single short beep */
            beep(1, 100, 100);
            break;
    }
}