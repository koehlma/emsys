#ifndef EPUCK_PATHFINDER_H
#define EPUCK_PATHFINDER_H

#include <hal/hal.h>
#include "map.h"
#include "sensors.h"
#include "bellman-ford/bellman-ford.h"

/* Caller must allocate space for MAX_PATH_LENGTH instances of Position. */

extern const Position INVALID_POS;

typedef struct PathFinderInputs {
    Map* map;
    double dest_x;
    double dest_y;
    unsigned int compute;
    unsigned int step_complete;
    unsigned int step_see_obstacle;
    unsigned int is_victim;
} PathFinderInputs;

typedef struct PathFinderLocals {
    Position path[MAX_PATH_LENGTH];
    int path_index; /* Must be signed */
    int state;
} PathFinderLocals;

typedef struct PathFinderState {
    PathFinderLocals locals;
    Position next;
    unsigned int no_path;
    unsigned int path_completed;
    unsigned int drive;
    unsigned int backwards;
} PathFinderState;

void pf_reset(PathFinderState* pf);
void pf_step(PathFinderInputs* inputs, PathFinderState* pf, Sensors* sens);

/* Only for testing */
int pf_find_path(Position position, Position goal, Map *map, Position *path);

#endif /*EPUCK_PATHFINDER_H*/
