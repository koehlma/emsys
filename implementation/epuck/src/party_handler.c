#include <hal/hal.h>
#include "party_handler.h"

#define SOFT_LED_DIST (3)
#define HARD_LED_DIST (2)

void set_leds(int state);

void phandler_reset(PHandlerState* state) {
    state->soft_state = state->hard_state = 0;
}

void phandler_step(PHandlerInput* input, PHandlerState* state) {
    unsigned int led_index;
    int on;
    if(input->party_hard) { /* priority on party hard */
        state->soft_state = 0;
        state->hard_state = (state->hard_state + 1) % HARD_LED_DIST;
        set_leds(state->hard_state);
    } else if(input->party_soft) {
        state->hard_state = 0;
        state->soft_state = (state->soft_state + 1) % SOFT_LED_DIST;
        set_leds(state->soft_state);
    }
}

void set_leds(int state) {
    unsigned int led_index;
    int on;
    for(led_index = 1; led_index < 8; ++led_index) {
        on = (led_index % state == 0);
        hal_set_led(led_index, (unsigned int) on);
    }
}
