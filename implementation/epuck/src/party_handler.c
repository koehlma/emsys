#include <hal/hal.h>
#include "party_handler.h"
#include "state-machine-common.h"

#define SOFT_LED_DIST (2)
#define HARD_LED_DIST (3)
#define CHANGE_SOFT_TIME_INTERVAL (1)
#define CHANGE_HARD_TIME_INTERVAL (0.25)

void set_leds(unsigned int state, unsigned int dist);

void phandler_reset(PHandlerState* state) {
    state->soft_state = state->hard_state = 0;
    state->entry = 0;
}

void phandler_step(PHandlerInput* input, PHandlerState* state) {
    double interval = input->party_hard ? CHANGE_HARD_TIME_INTERVAL : CHANGE_SOFT_TIME_INTERVAL;
    if(!smc_time_passed_p(state->entry, interval)) {
        return;
    }
    state->entry = hal_get_time();
    if(input->party_hard) { /* priority on party hard */
        state->soft_state = 0;
        state->hard_state = (state->hard_state + 1) % HARD_LED_DIST;
        set_leds(state->hard_state, HARD_LED_DIST);
    } else if(input->party_soft) {
        state->hard_state = 0;
        state->soft_state = (state->soft_state + 1) % SOFT_LED_DIST;
        set_leds(state->soft_state, SOFT_LED_DIST);
    }
}

void set_leds(unsigned int state, unsigned int dist) {
    unsigned int led_index;
    int on;
    for(led_index = 0; led_index < 8; ++led_index) {
        on = (led_index % dist == state);
        hal_set_led(led_index, (unsigned int) on);
    }
}
