#include "log_config.h"
#include "moderator.h"
#include "state-machine-common.h"
#include "t2t.h"
#include "t2t-parse.h"

enum {
    MOD_STATE_SEARCHING,
    MOD_STATE_WAITING_BIDDING,
    MOD_STATE_WAITING_WATCHING,
    MOD_STATE_RESCUEING_LOUD,
    MOD_STATE_RESCUEING_SILENT,
    MOD_STATE_DEAD
};

#define T2T_HEARTBEAT_TIMEOUT_SECS 25
/* Pathfinder needs at least 9.6 seconds, so the timeout definitely shouldn't be shorter than that. */
typedef char check_heartbeat_timeout[(T2T_HEARTBEAT_TIMEOUT_SECS > 10 + T2T_HEARTBEAT_PERIOD_SECS) ? 1 : -1];
typedef char check_moderator_constant[(MOD_STATE_RESCUEING_SILENT == 4) ? 1 : -1];

void mod_reset(ModState* mod) {
    mod->may_run_p = 1;
    mod->victim.x = -1;
    mod->victim.y = -1;
    mod->found_victim_xy = 0;
    mod->locals.time_entered = 0;
    mod->locals.state = MOD_STATE_SEARCHING;
    mod->locals.sent_iteration = -1;
}

void mod_step(ModInputs* inputs, ModState* mod, Sensors* sens) {
    int old_state;

    /* Make sure we never lag too far behind: */
    if (mod->locals.sent_iteration < inputs->t2t_data->newest_theirs - 1) {
        mod->locals.sent_iteration = inputs->t2t_data->newest_theirs - 1;
    }
    /* Obey Pickup Artist: */
    if ((inputs->give_up || inputs->t2t_data->need_to_die)
            && mod->locals.state != MOD_STATE_DEAD) {
        #ifdef LOG_TRANSITIONS_MOD
        hal_print("mod:*->dead");
        #endif
        mod->locals.state = MOD_STATE_DEAD;
    }

    old_state = mod->locals.state;
    switch (mod->locals.state) {
    case MOD_STATE_SEARCHING:
        if (inputs->t2t_data->newest_theirs >= 0) {
            /* Other Tin Bot was much faster. */
            mod->locals.sent_iteration = 0;
            mod->locals.state = MOD_STATE_WAITING_WATCHING;
            mod->may_run_p = 0;
            smc_halt();
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:search->wait");
            #endif
        } else if (inputs->found_victim_xy) {
            t2t_send_found_xy(inputs->own_victim.x, inputs->own_victim.y, 0);
            mod->locals.sent_iteration = 0;
            mod->locals.state = MOD_STATE_WAITING_BIDDING;
            mod->may_run_p = 0;
            smc_halt();
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:search->bid");
            #endif
        }
        break;
    case MOD_STATE_WAITING_BIDDING:
        if (inputs->t2t_data->owning_xy_p) {
            /* "Whee!  My turn!" */
            mod->locals.state = MOD_STATE_RESCUEING_LOUD;
            mod->may_run_p = 1;
            mod->victim.x = inputs->t2t_data->seen_x;
            mod->victim.y = inputs->t2t_data->seen_y;
            mod->found_victim_xy = 1;
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:bid->rescue");
            #endif
        } else if (inputs->t2t_data->newest_theirs >= mod->locals.sent_iteration) {
            /* Other Tin Bot was faster. */
            mod->locals.sent_iteration = inputs->t2t_data->newest_theirs;
            mod->locals.state = MOD_STATE_WAITING_WATCHING;
            /* Should already be 0 anyway. */
            mod->may_run_p = 0;
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:bid->wait");
            #endif
        }
        break;
    case MOD_STATE_WAITING_WATCHING:
        /* We don't have strong time guarantees on the main loop, so
         * *first* check whether there is a heartbeat "somewhere" in the
         * buffer, and only then check the time.
         * t2t.c guarantees us that no heartbeat is lost.  The exact time
         * of the beat is lost, but we know that it was "at some point
         * during the last main loop". */
        if (inputs->t2t_data->seen_beat) {
            /* Reset timer manually, as we can't change state. */
            mod->locals.time_entered = hal_get_time();
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:wait->wait (HB)");
            #endif
        } else if (smc_time_passed_p(mod->locals.time_entered, T2T_HEARTBEAT_TIMEOUT_SECS)) {
            /* They died.  Acknowledge the dead, and use the coordinates they
             * sent.  We do this because there's no guarantee that *we* ever
             * triangulated VICTOR successfully, and also would like to behave
             * "all in sync". */
            ++mod->locals.sent_iteration;
            t2t_send_found_xy(inputs->t2t_data->seen_x, inputs->t2t_data->seen_y, mod->locals.sent_iteration);
            mod->locals.state = MOD_STATE_WAITING_BIDDING;
            mod->may_run_p = 0;
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:wait->bid");
            #endif
        }
        break;
    case MOD_STATE_RESCUEING_LOUD:
        if (!inputs->t2t_data->owning_xy_p) {
            /* Some other Tin Bot went wrong.  Stop. */
            smc_halt();
            mod->locals.state = MOD_STATE_DEAD;
            mod->may_run_p = 0;
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:rescue->dead");
            #endif
        } else if (sens->victim_attached) {
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:rescue->docked");
            #endif
            mod->locals.state = MOD_STATE_RESCUEING_SILENT;
            t2t_send_docked();
        } else if (smc_time_passed_p(mod->locals.time_entered, T2T_HEARTBEAT_PERIOD_SECS)) {
            mod->locals.time_entered = hal_get_time();
            t2t_send_heartbeat();
            #ifdef LOG_TRANSITIONS_MOD
            hal_print("mod:rescue->rescue (HB)");
            #endif
        }
        break;
    case MOD_STATE_RESCUEING_SILENT:
        /* Nothing to do here. */
        break;
    case MOD_STATE_DEAD:
        /* Nothing to do here. */
        break;
    }

    if (mod->locals.state != old_state) {
        mod->locals.time_entered = hal_get_time();
    }
}
