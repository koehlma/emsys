#ifndef EPUCK_PATHFINDER_H
#define EPUCK_PATHFINDER_H

#include <stdint.h>

#include "bellman-ford/bellman-ford.h"
#include "hal.h"
#include "map.h"
#include "sensors.h"

typedef struct PathFinderInputs {
    ExactPosition dest;
    unsigned int compute;
    unsigned int step_complete;
    unsigned int step_see_obstacle;
    unsigned int is_victim;
} PathFinderInputs;

typedef struct PathFinderLocals {
    BellmanFord bf_state;
    int state;
    /* path_index special values:
     * -1 There is no path
     * -2 Virtual waypoint that is immediately reached,
     *    whose successor is bf_state->init_v */
    int16_t next_v;
} PathFinderLocals;

typedef struct PathFinderState {
    PathFinderLocals locals;
    ExactPosition next;
    unsigned int no_path;
    unsigned int path_completed;
    unsigned int drive;
    unsigned int backwards;
} PathFinderState;

void pf_reset(PathFinderState* pf);
void pf_step(PathFinderInputs* inputs, PathFinderState* pf, Sensors* sens);

/* Only for testing */
/* FIXME: Change position to "ExactPosition init" */
void pf_find_path(Position position, ExactPosition goal, BellmanFord* bf_state);

#endif /*EPUCK_PATHFINDER_H*/
