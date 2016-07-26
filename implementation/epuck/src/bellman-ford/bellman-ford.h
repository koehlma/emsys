#ifndef EPUCK_BELLMAN_FORD_H
#define EPUCK_BELLMAN_FORD_H

#include <stdint.h>

#include "map.h"

#define STEPPING_DIST (8)
#define VERTICES_PER_COL ((MAP_MAX_HEIGHT + STEPPING_DIST - 1) / STEPPING_DIST)
#define VERTICES_PER_ROW ((MAP_MAX_WIDTH  + STEPPING_DIST - 1) / STEPPING_DIST)
#define NUM_VERTICES (VERTICES_PER_COL * VERTICES_PER_ROW)

typedef struct BellmanFord {
    /* --- Input --- */
    ExactPosition init;
    ExactPosition goal;
    /* --- Locals --- */
    /* Distance to the goal. */
    int16_t distances[NUM_VERTICES];
    /* --- Output --- */
    /* If there is no path, init_v will be set to -1.
     * Otherwise, it's the first waypoint. */
    int16_t init_v;
    /* assert(succ[goal_v] == -1); */
    int16_t goal_v;
    /* Next waypoint towards the goal. */
    int16_t succ[NUM_VERTICES];
} BellmanFord;

/* Not the most efficient realization */
unsigned int bf_adjacent_p(ExactPosition v, ExactPosition u);

void find_path(BellmanFord* state);

ExactPosition bf_v2pos(BellmanFord* state, int16_t v);

/* Only for testing and benchmarking */
unsigned int bellman_ford_cycle(BellmanFord* state);
unsigned int init_bellman_ford(BellmanFord* state);

#endif /* EPUCK_BELLMAN_FORD_H */
