#ifndef EPUCK_BELLMAN_FORD_H
#define EPUCK_BELLMAN_FORD_H

#include "map.h"

#define STEPPING_DIST (8)
#define VERTICES_PER_COL ((MAP_MAX_HEIGHT + STEPPING_DIST - 1) / STEPPING_DIST)
#define VERTICES_PER_ROW ((MAP_MAX_WIDTH  + STEPPING_DIST - 1) / STEPPING_DIST)
#define NUM_VERTICES (VERTICES_PER_COL * VERTICES_PER_ROW)
#define MAX_PATH_LENGTH (NUM_VERTICES + 1)

typedef struct BellmanFordLocals {
    int distances[NUM_VERTICES];
    int pred[NUM_VERTICES];
} BellmanFordLocals;

typedef struct BellmanFord {
    int (*adj)(Position v, Position u, Map* map); /* not the most efficient realization */
    Position init;
    ExactPosition goal;
    Map* map;
    Position* path;
} BellmanFord;

void find_path(BellmanFord* state);

#endif /* EPUCK_BELLMAN_FORD_H */
