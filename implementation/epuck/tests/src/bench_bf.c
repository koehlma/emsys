#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "bellman-ford/bellman-ford.h"
#include "hal.h"
#include "map_heap.h"

int main() {
    int i;
    MapContainer mc;
    time_t bench_time;
    BellmanFord* state = malloc(sizeof(BellmanFord));

    state->init.x = 5;
    state->init.y = 6;
    state->goal.x = 50;
    state->goal.y = 60;
    init_bellman_ford(state);

    /* Setting everything up */
    mc.accu = map_heap_alloc(MAP_MAX_WIDTH, MAP_MAX_HEIGHT);
    mc.prox = NULL;
    map_heap_container = &mc;

    /* Run, bitch, run! */
    printf("Yo, starting.\n");
    bench_time = time(NULL);
    for (i = 0; i < 1000000; i++) {
        bellman_ford_cycle(state);
    }
    bench_time = time(NULL) - bench_time;
    printf("Done after %ld (ignore this: %d).\n", bench_time, state->succ[5]);

    /* Free everything */
    map_heap_free(mc.accu);
    free(state);

    return 0;
}
