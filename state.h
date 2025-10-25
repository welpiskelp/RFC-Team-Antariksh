/*************************************************************************
COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Rik Seth

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission
   from Team Antariksh.

*************************************************************************/

#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stdint.h>

#define FLAG_INITIALIZING_BIT 0x01
#define FLAG_STANDBY_BIT 0x02
#define FLAG_LIFTOFF_CONFIRMED_BIT 0x04
#define FLAG_APOGEE_REACHED_BIT 0x08
#define FLAG_MAIN_EJECTED_BIT 0x10
#define FLAG_DESCENT_STARTED_BIT 0x20
#define FLAG_PARACHUTE_EJECTED_BIT 0x40
#define FLAG_TOUCHDOWN_CONFIRMED_BIT 0x80

typedef int state_flags_t;
typedef enum state {
    STATE_INITIALIZING = 0,
    STATE_STANDBY = 1,
    STATE_LIFTOFF_CONFIRMED = 2,
    STATE_APOGEE_REACHED = 3,
    STATE_MAIN_EJECTED = 4,
    STATE_DESCENT_STARTED = 5,
    STATE_PARACHUTE_EJECTED = 6,
    STATE_TOUCHDOWN_CONFIRMED = 7,
} state_t;

void update_state(state_flags_t flags, state_t *ps, state_t *ns, bool *changed);

#endif
