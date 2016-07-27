#include <stdio.h>
#include <stdlib.h> /* malloc, free */

#include "hal.h"
#include "map_heap.h"
#include "path-finder.h"

int main(void){
	int i;
    ExactPosition start, goal;
    Map* map;
    MapContainer mc;
    BellmanFord* bf_state;

    bf_state = malloc(sizeof(BellmanFord));

    /* Build a map */
    map_heap_container = &mc;
    map = map_heap_alloc(MAP_MAX_WIDTH, MAP_MAX_HEIGHT);
    mc.accu = map;
    for(i = 0; i <= 10; ++i) {
        map_set_field(map, 12, i, FIELD_WALL);
    }

    start.x = 5.3;
    start.y = 5.1;
    goal.x = 17.1;
    goal.y = 6.1;

	printf("Starting search\n");
	pf_find_path(start, goal, bf_state);
    assert(bf_state->init_v != -1);

    printf("Found path:\n");
    i = 0;
    while (bf_state->init_v != -1) {
        ExactPosition buf;
        buf = bf_v2pos(bf_state, bf_state->init_v);
		printf("-> (%.1f, %.1f)", buf.x, buf.y);
        bf_state->init_v = bf_state->succ[bf_state->init_v];
        assert(++i < 30);
	}
    printf("\n\tEVALUATE BY HAND!\n");

    free(bf_state);
    map_heap_container = NULL;
    map_heap_free(map);
    free_printbuf();

	return 0;
}
