/*************************************************************************
COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Rik Seth

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission
   from Team Antariksh.

*************************************************************************/

#include "state.h"
#include <stddef.h>

void update_state(state_flags_t flags, state_t *ps, state_t *ns, bool *changed) {
    if (ps == NULL) {
        *ns = STATE_STANDBY;
    }

    *changed = false;
    switch (*ps) {
        case STATE_INITIALIZING: {
            if (flags & FLAG_STANDBY_BIT) {
                *ns = STATE_STANDBY;
                *changed = true;
            }
            break;
        }
        case STATE_STANDBY: {
            if (flags & FLAG_LIFTOFF_CONFIRMED_BIT) {
                *ns = STATE_LIFTOFF_CONFIRMED;
                *changed = true;
            }
            break;
        }
        case STATE_LIFTOFF_CONFIRMED: {
            if (flags & FLAG_APOGEE_REACHED_BIT) {
                *ns = STATE_APOGEE_REACHED;
                *changed = true;
            }
            break;
        }
        case STATE_APOGEE_REACHED: {
            if (flags & FLAG_MAIN_EJECTED_BIT) {
                *ns = STATE_MAIN_EJECTED;
                *changed = true;
            }
            break;
        }
        case STATE_MAIN_EJECTED: {
            if (flags & FLAG_DESCENT_STARTED_BIT) {
                *ns = STATE_DESCENT_STARTED;
                *changed = true;
            }
            break;
        }
        case STATE_DESCENT_STARTED: {
            if (flags & FLAG_PARACHUTE_EJECTED_BIT) {
                *ns = STATE_PARACHUTE_EJECTED;
                *changed = true;
            }
            break;
        }
        case STATE_PARACHUTE_EJECTED: {
            if (flags & FLAG_TOUCHDOWN_CONFIRMED_BIT) {
                *ns = STATE_TOUCHDOWN_CONFIRMED;
                *changed = true;
            }
            break;
        }
        case STATE_TOUCHDOWN_CONFIRMED:
            break;
    }
}
