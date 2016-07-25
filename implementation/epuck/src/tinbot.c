/*
 * Tin Bot Controller Software
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "hal.h"
#include "map.h"
#include "rhr.h"
#include "sensors.h"
#include "tinbot.h"

void update_proximity(TinBot* tinbot, double proximity[8]) {
    unsigned int number;
    for (number = 0; number < 8; number++) {
        /* Q: Why doesn't this do any conversion?
         * A: Because the incoming values already are in cm. */
        tinbot->sens.proximity[number] = proximity[number];
    }
}

void update_ir(TinBot* tinbot, int ir[6]) {
    unsigned int i;
    for (i = 0; i < 6; ++i) {
        tinbot->sens.ir[i] = ir[i];
    }
    hal_set_led(1, (unsigned int) ir[0]);
    hal_set_led(2, (unsigned int) ir[1]);
    hal_set_led(3, (unsigned int) ir[2]);
    hal_set_led(5, (unsigned int) ir[3]);
    hal_set_led(6, (unsigned int) ir[4]);
    hal_set_led(7, (unsigned int) ir[5]);
}

void update_victim_pickup(TinBot* tinbot, unsigned int grabbed) {
    tinbot->sens.victim_attached = !!grabbed;
}

void update_lps(TinBot* tinbot, double x, double y, double phi) {
    tinbot->sens.lps.x = x;
    tinbot->sens.lps.y = y;
    tinbot->sens.lps.phi = phi;
}


/* Mode - Full */
static void setup_full(TinBot* tinbot) {
    hal_print("Tin Bot Setup: FULL");
    controller_reset(&tinbot->controller, &tinbot->sens);
}

static void loop_full(TinBot* tinbot) {
    controller_step(&tinbot->controller, &tinbot->sens);
}


/* Mode - MapOnly */
static void setup_maponly(TinBot* tinbot) {
    hal_print("Tin Bot Setup: Map Only");
    approx_reset(&tinbot->controller.approx, &tinbot->sens);
    proximity_reset(&tinbot->controller.prox_map, &tinbot->sens);
}

static void loop_maponly(TinBot* tinbot) {
    approx_step(&tinbot->controller.approx, &tinbot->sens);
    proximity_step(&tinbot->controller.prox_map, &tinbot->sens);
}


/* Mode - RHR */
static void setup_rhr(TinBot* tinbot) {
    hal_print("Tin Bot Setup: RHR");
    approx_reset(&tinbot->controller.approx, &tinbot->sens);
    rhr_reset(&tinbot->controller.rhr);
}

static void loop_rhr(TinBot* tinbot) {
    rhr_step(&tinbot->controller.rhr, &tinbot->sens);
}


/* Mode - VICDIR */
static void setup_vicdir(TinBot* tinbot) {
    hal_print("Tin Bot Setup: VicDir");
    vd_reset(&tinbot->controller.vic_dir);
    approx_reset(&tinbot->controller.approx, &tinbot->sens);
}

static void loop_vicdir(TinBot* tinbot) {
    approx_step(&tinbot->controller.approx, &tinbot->sens);

    vd_step(&tinbot->sens.t2t.fixdir, &tinbot->controller.vic_dir, &tinbot->sens);
    hal_debug_out(DEBUG_CAT_VD_STATE, tinbot->controller.vic_dir.locals.state);
    hal_debug_out(DEBUG_CAT_VD_VICTIM_FOUND, tinbot->controller.vic_dir.victim_found);
    hal_debug_out(DEBUG_CAT_VD_VICTIM_PHI, tinbot->controller.vic_dir.victim_phi);
    hal_debug_out(DEBUG_CAT_VD_GIVE_UP, tinbot->controller.vic_dir.give_up);
    hal_debug_out(DEBUG_CAT_VD_ON_PERCENTAGE, tinbot->controller.vic_dir.locals.counter_on * 1.0 /
                                              tinbot->controller.vic_dir.locals.counter_total);
    hal_debug_out(DEBUG_CAT_VD_AVG_ANGLE,
                  tinbot->controller.vic_dir.locals.weighted_sum / tinbot->controller.vic_dir.locals.counter_on);
}


/* Mode - mergeonly */
static void setup_mergeonly(TinBot* tinbot) {
    (void)tinbot;
    hal_print("Tin Bot Setup: mergeonly");
}

static void loop_mergeonly(TinBot* tinbot) {
    static const long iterations = 10000;
    long i = 0;
    hal_time time = hal_get_time();
    (void)tinbot;
    do {
        map_merge(map_get_accumulated(), 4, 2, map_get_proximity());
    } while (++i < iterations);
    time = hal_get_time() - time;
    sprintf(hal_get_printbuf(), "merge_only: avg over %ld iter: %.3f us/iter",
        iterations,
        time * 1000.0 /* 1000 us/ms */ / iterations);
    hal_print(hal_get_printbuf());
}


/* Mode - pathfin */
static unsigned int pathfin_origin_resetted = 0;

static void setup_pathfin(TinBot* tinbot) {
    hal_print("Tin Bot Setup: Path Finder (20,30), somewhat");
    controller_reset(&tinbot->controller, &tinbot->sens);
    pathfin_origin_resetted = 0;

    /* First, persuade the Moderator that we're already rescuing the victim
     * all along, and that this is fine. */
    tinbot->controller.moderator.locals.state = 4;
    tinbot->rx_buffer.moderate.owning_xy_p = 1;
    /* Call ProxFilter, for fun.
     * Also, disable TCE's interrupt. */
    tinbot->controller.moderator.found_victim_xy = 1;
    /* approx_step can run freely, as approx_reset was called by
     * controller_reset. */

    /* Next, Blind Cop has lots of internal state. */
    tinbot->controller.blind.locals.state_big = 1;
    tinbot->controller.blind.locals.state_leaf = 5;
    /* The rest should be propagated automatically. */
}

static void loop_pathfin(TinBot* tinbot) {
    tinbot->sens.victim_attached = 1;
    if (!pathfin_origin_resetted) {
        tinbot->sens.lps.x = 20.0;
        tinbot->sens.lps.y = 30.0;
        pathfin_origin_resetted = 1;
    }
    controller_step(&tinbot->controller, &tinbot->sens);
}


static void setup_bluetooth_test(TinBot* tinbot) {

}

static void loop_bluetooth_test(TinBot* tinbot) {
    unsigned int i;
    hal_print("Bluetooth Test");
    for (i = 0; i < 20; i++) {
        hal_print("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz1234567890");
    }
}


static TinMode modes[7] = {
        {setup_full,      loop_full},
        {setup_rhr,       loop_rhr},
        {setup_mergeonly, loop_mergeonly},
        {setup_vicdir,    loop_vicdir},
        {setup_maponly,   loop_maponly},
        {setup_pathfin,   loop_pathfin},
        {setup_bluetooth_test, loop_bluetooth_test}
};

void setup(TinBot* tinbot) {
    tinbot->sens.victim_attached = 0;
    t2t_data_init(&tinbot->rx_buffer);
    t2t_data_init(&tinbot->sens.t2t);
    modes[tinbot->mode].setup(tinbot);
    map_clear(map_get_accumulated());
    map_clear(map_get_proximity());
}

void loop(TinBot* tinbot) {
    if (tinbot->sens.t2t.send_buf) {
        hal_print("INTERRUPT SENT:");
        hal_print(tinbot->sens.t2t.send_buf);
    }
    if (tinbot->sens.t2t.send_lost) {
        hal_print("(+ lost message(s))");
    }
    modes[tinbot->mode].loop(tinbot);
}

void set_mode(TinBot* tinbot, unsigned int mode) {
    tinbot->mode = mode;
}
