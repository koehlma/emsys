#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "hal/hal.h"
#include "log_config.h"
#include "pi.h"
#include "sensors.h"
#include "state-machine-common.h"
#include "victim-direction.h"
#include "t2t-parse.h"

/* Invariants:
 * state==VD_done \iff precisely one output flag is set
 * state!=VD_done \iff none of the output flags are set */
enum {
    VD_off,
    VD_running,
    VD_wait_revolved,
    VD_wait_judgement,
    VD_done
};

/* We know that the IR sensors should at *least*
 * have a sensibility of this many degrees. */
static const double VD_MIN_ON = 9.0 / 360.0;
/* Half the "IR pause time", plus the full I2C delay: */
#define IR_DELAY_TIME (150 / 2 + 50)
#define IR_COMPLETION_TIME ((hal_time)(1000 * 2 * M_PI / SMC_ROT_PER_SEC))
#define VD_WAIT_ON_LPS_SECS 2

void vd_reset(VDState* vd){
    vd->locals.state = VD_off;
    vd->victim_found = 0;
    vd->victim_phi = 0;
    vd->give_up = 0;
    vd->locals.counter_total = 1;
    vd->locals.counter_on = 0;
    vd->locals.time_begin = 0;
    vd->locals.weighted_sum = 0;
    vd->locals.gap_phi = -1;
}

typedef char check_num_ir_even[(NUM_IR % 2 == 0) ? 1 : -1];

static void entry_start(VDState* vd, Sensors* sens) {
    int i, have_id;
    #ifdef LOG_TRANSITIONS_VICDIR
    hal_print("VD:start");
    #endif
    vd->locals.state = VD_running;
    have_id = 0;
    for(i = 0; i < NUM_IR; ++i) {
        if (sens->ir[i]) {
            int sensor_id = (i + NUM_IR / 2) % NUM_IR;
            have_id = 1;
            vd->locals.gap_phi = ir_sensor_angle[sensor_id];
            break;
        }
    }
    if (have_id == 0) {
        vd->locals.state = VD_done;
        vd->give_up = 1;
    } else {
        smc_rot_left();
        vd->locals.time_begin = hal_get_time();
    }
}

static void compute_result(VDState* vd, Sensors* sens) {
    double eff_opening;

    if (vd->locals.counter_total < 200) {
        /* Wat. */
        #ifdef LOG_TRANSITIONS_VICDIR
        sprintf(hal_get_printbuf(), "VD:too few total samples (%lu)", vd->locals.counter_total);
        hal_print(hal_get_printbuf());
        #endif
        vd->give_up = 1;
        vd->locals.state = VD_wait_judgement;
        return;
    }

    eff_opening = vd->locals.counter_on * 1.0 / (NUM_IR * vd->locals.counter_total);
    if (eff_opening < VD_MIN_ON) {
        /* There is no spoon.  And no VICTOR. */
        #ifdef LOG_TRANSITIONS_VICDIR
        sprintf(hal_get_printbuf(), "VD:too small effective opening (%.1f deg)", eff_opening);
        hal_print(hal_get_printbuf());
        #endif
        vd->give_up = 1;
        vd->locals.state = VD_done;
        return;
    }

    vd->victim_phi = sens->current.phi + vd->locals.gap_phi
        + (2 * M_PI + vd->locals.weighted_sum / vd ->locals.counter_on);
    vd->victim_phi = fmod(vd->victim_phi + 2 * M_PI, 2 * M_PI);

    #ifdef LOG_TRANSITIONS_VICDIR
    {
        if (-9 < vd->victim_phi && vd->victim_phi < 99) {
            sprintf(hal_get_printbuf(), "VD:phi=%.3f", vd->victim_phi);
        } else {
            sprintf(hal_get_printbuf(), "VD:phi=inf");
        }
        hal_print(hal_get_printbuf());
    }
    #endif
    send_found_phi(sens->current.x, sens->current.y, vd->victim_phi);
}

void vd_step(T2TData_VicFix* input, VDState* vd, Sensors* sens, ProxMapState* prox_map){
    switch(vd->locals.state) {
        case VD_off:
            entry_start(vd, sens);
            break;
        case VD_running:
            {
                const hal_time rot_msecs = hal_get_time() - vd->locals.time_begin;
                if (rot_msecs < 2 * IR_DELAY_TIME) {
                    /* Can't use the sample yet, and definitely didn't do a full
                     * rotation yet.  For symmetry, make sure the "gap of willful
                     * ignorance" is centered on the 'gap_phi' */
                    break;
                }
                if (rot_msecs < IR_COMPLETION_TIME) {
                    const double rot_angle =
                        (rot_msecs - IR_DELAY_TIME) * SMC_ROT_PER_SEC / 1000.0;
                    int i;
                    vd->locals.counter_total += 1;
                    for (i = 0; i < NUM_IR; ++i) {
                        if (!sens->ir[i]) {
                            continue;
                        }
                        vd->locals.counter_on += 1;
                        vd->locals.weighted_sum +=
                            fmod(rot_angle + ir_sensor_angle[i] - vd->locals.gap_phi + 2 * M_PI, 2 * M_PI);
                    }
                }
                proximity_clear_around_us(sens, prox_map);
                if (rot_msecs >= IR_COMPLETION_TIME + 30 /* And a bit more: 0.65 degree */) {
                    #ifdef LOG_TRANSITIONS_VICDIR
                    hal_print("VD:done running");
                    #endif
                    vd->locals.time_begin = hal_get_time();
                    vd->locals.state = VD_wait_revolved;
                    smc_halt();
                }
            }
            break;
        case VD_wait_revolved:
            if (smc_time_passed_p(vd->locals.time_begin, VD_WAIT_ON_LPS_SECS)) {
                #ifdef LOG_TRANSITIONS_VICDIR
                hal_print("VD:done waiting for lps");
                #endif
                vd->locals.state = VD_wait_judgement;
                compute_result(vd, sens);
            }
            break;
        case VD_wait_judgement:
            if (input->have_incoming_fix) {
                #ifdef LOG_TRANSITIONS_VICDIR
                hal_print("VD:got judgement");
                #endif
                if (input->acceptable) {
                    #ifdef LOG_TRANSITIONS_VICDIR
                    hal_print("VD:accept->found");
                    #endif
                    vd->victim_phi = input->phi_correct;
                    vd->victim_found = 1;
                } else {
                    #ifdef LOG_TRANSITIONS_VICDIR
                    hal_print("VD:inacceptable->use LPS' phi");
                    #endif
                    /* vd->give_up = 1; Tough luck. */
                    vd->victim_phi = input->phi_correct;
                    vd->victim_found = 1;
                }
            }
        case VD_done:
            break;
        default:
            hal_print("Invalid state in victim direction. VICTOR, where are you??");
            assert(0);
            vd_reset(vd);
            break;
    }
}
