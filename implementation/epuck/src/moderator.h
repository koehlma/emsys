#ifndef EPUCK_MODERATOR_H
#define EPUCK_MODERATOR_H

#include "hal.h"
#include "map.h"
#include "sensors.h"
#include "t2t-data.h"

typedef struct ModInputs {
    T2TData_Moderate* t2t_data;
    ExactPosition own_victim;
    unsigned int found_victim_xy;
    unsigned int give_up;
} ModInputs;

typedef struct ModLocals {
    hal_time time_entered;
    int state;
    int sent_iteration; /* Must be signed */
} ModLocals;

typedef struct ModState {
    ModLocals locals;
    unsigned int may_run_p;
    /* Use the same position on *every* Tin Bot.
     * That's necessary in the following edge case:
     * - we didn't successfully triangulate VICTOR
     * - someone else claims they did
     * - they die
     * - we win the bidding to rescue VICTOR, but don't
     *   know any position of our own.
     * Also, it's nice to know that all Bots are "synchronized". */
    ExactPosition victim;
    unsigned int found_victim_xy;
} ModState;

void mod_reset(ModState* mod);
void mod_step(ModInputs* inputs, ModState* mod, Sensors* sens);

#endif
