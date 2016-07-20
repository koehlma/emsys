#include <math.h>

#include "hal.h"
#include "pi.h" /* M_PI */
#include "path-exec.h"
#include "state-machine-common.h"
#include "sensors.h"

#define TOLERANCE_ANGLE (5 * M_PI / 180)

enum PE_STATES {
    PE_inactive,
    PE_compute,
    PE_rotate, /* turn to */
    PE_rot_wait_for_LPS,
    PE_drive, /* tune in */
    PE_profit /* and drop out */
};
/* static_assert(sizeof(enum PE_STATES) == sizeof(PathExecLocals::state),
 *               "Bad things"); */
typedef char check_pe_states_size[
    (sizeof(enum PE_STATES) == sizeof(int)) ? 1 : -1];

static const double PE_MAX_STRAY = 10;

void pe_reset(PathExecState* pe) {
    pe->done = 0;
    pe->see_obstacle = 0;
    pe->locals.state = PE_inactive;
    pe->locals.approx_rot_speed = SMC_ROT_PER_SEC;
    /* No further initialization needed because PE_inactive doesn't
       read from locals.time_entered or locals.start_* or locals.need_* */
}

#define PE_CRASH_TOLERANCE 1.0

static unsigned int near_crash_p(PathExecInputs* inputs, Sensors* sens) {
    if (inputs->backwards) {
        return sens->proximity[PROXIMITY_M_150] < PE_CRASH_TOLERANCE
            || sens->proximity[PROXIMITY_P_150] < PE_CRASH_TOLERANCE;
    } else {
        return sens->proximity[PROXIMITY_M_20] < PE_CRASH_TOLERANCE
            || sens->proximity[PROXIMITY_P_20] < PE_CRASH_TOLERANCE;
    }
}

void pe_step(PathExecInputs* inputs, PathExecState* pe, Sensors* sens) {
    PathExecLocals* l;
    int old_state;

    l = &pe->locals;
    old_state = l->state;

    if (!inputs->drive) {
        /* We just got notified that we shouldn't be running anymore. */
        l->state = PE_inactive;
        /* It's okay to 'halt()' many times per second. */
        smc_halt();
        /* Don't set 'time_entered'; not needed. */
        return;
    }

    switch (l->state) {
    case PE_inactive:
        if(inputs->drive) {
            l->state = PE_compute;
        }
        break;
    case PE_compute:
        {
            double start_dir;
            double target_dir;
            l->start_x = sens->current.x;
            l->start_y = sens->current.y;
            start_dir = sens->current.phi;
            target_dir = atan2(inputs->next_y - l->start_y,
                               inputs->next_x - l->start_x);
            if(inputs->backwards) {
                target_dir += M_PI;
            }
            l->init_dir = start_dir;
            /* Rotation in radians: */
            l->need_rot = fmod(target_dir - start_dir + M_PI, 2 * M_PI) - M_PI;
            /* Rotation in seconds: */
            l->need_rot /= l->approx_rot_speed;
            l->normal_x = -(inputs->next_y - l->start_y);
            l->normal_y =   inputs->next_x - l->start_x;
            l->need_dist = l->normal_x * l->normal_x + l->normal_y * l->normal_y;
            l->need_dist = sqrt(l->need_dist);
            l->normal_x /= l->need_dist;
            l->normal_y /= l->need_dist;
            l->need_dist /= SMC_MV_PER_SEC;

            /* Code from the transitions */
            l->state = PE_rotate;
            if (l->need_rot < 0) {
                l->need_rot *= -1;
                smc_rot_right();
            } else {
                smc_rot_left();
            }
        }
        break;
    case PE_rotate:
         if (smc_time_passed_p(l->time_entered, l->need_rot)) {
            smc_halt();
            l->time_entered = hal_get_time();
            l->state = PE_rot_wait_for_LPS;
        }
        break;
    case PE_rot_wait_for_LPS:
        if(smc_time_passed_p(l->time_entered, 2.2)) {
            double actually_rotated_per_sec, actually_rotated;
            actually_rotated = fabs(sens->current.phi - l->init_dir);
            actually_rotated = fmod(actually_rotated, 2 * M_PI);
            if (actually_rotated <= TOLERANCE_ANGLE) {
                l->state = PE_drive;
                if (inputs->backwards) {
                    smc_move();
                } else {
                    smc_move_back();
                }
            } else {
                actually_rotated_per_sec = actually_rotated / l->need_rot;
                l->approx_rot_speed = actually_rotated_per_sec;
                l->state = PE_compute;
            }
        }
        break;
    case PE_drive:
        /* FIXME: use filtered proximity data */
        if (near_crash_p(inputs, sens)) {
            smc_halt();
            l->state = PE_profit;
            pe->see_obstacle = 1;
            break;
        }
        {
            double stray;
            stray = 0;
            stray += (inputs->next_x - sens->current.x) * l->normal_x;
            stray += (inputs->next_y - sens->current.y) * l->normal_y;
            stray = fabs(stray);
            if (stray >= PE_MAX_STRAY) {
                /* Whoopsie daisy. */
                l->state = PE_compute;
                smc_halt();
                break;
            }
        }
        if (smc_time_passed_p(l->time_entered, l->need_dist)) {
            l->state = PE_profit;
            smc_halt();
            pe->done = 1;
        }
        break;
    case PE_profit:
        /* Nothing to do here. */
        break;
    default:
        /* Uhh */
        hal_print("PathExec illegal state?!  Resetting ...");
        assert(0);
        pe_reset(pe);
        smc_halt();
        break;
    }

    if (l->state != old_state) {
        l->time_entered = hal_get_time();
    }
}
