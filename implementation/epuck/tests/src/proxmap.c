#include <stdlib.h>
#include <stdio.h>

#include "hal.h"
#include "map.h"
#include "prox-map.h"
#include "map_heap.h"

int main() {
    int x, y;
    ProxMapState pm;
    Sensors sens;

    /* Initialize environment */
    map_heap_container = malloc(sizeof(MapContainer));
    map_heap_container->accu = NULL;
    map_heap_container->prox = map_heap_alloc(MAP_PROXIMITY_SIZE, MAP_PROXIMITY_SIZE);
    assert(map_heap_container->prox == map_get_proximity());
    assert(map_heap_container->prox != NULL);
    sens.proximity[0] = 4;
    sens.proximity[1] = 4;
    sens.proximity[2] = 4;
    sens.proximity[3] = 4;
    sens.proximity[4] = 4;
    sens.proximity[5] = 4;
    sens.proximity[6] = 4;
    sens.proximity[7] = 4;
    sens.current.x = 12;
    sens.current.y = 10;
    sens.current.phi = 0;

    /* Initialize ProxMap */
    proximity_reset(&pm, &sens);
    printf("ProxMap initialized to low_left=(%d,%d)\n", pm.lower_left.x, pm.lower_left.y);

    /* Run it: */
    proximity_step(&pm, &sens);

    /* Print it: */
    printf("ProxMap, low_left=(%d,%d) is:\n", pm.lower_left.x, pm.lower_left.y);
    for (y = MAP_PROXIMITY_SIZE; y > 0; --y) {
        for (x = 0; x < MAP_PROXIMITY_SIZE; ++x) {
            switch (map_get_field(map_get_proximity(), x, y - 1)) {
            case FIELD_UNKNOWN:
                printf(" .");
                break;
            case FIELD_FREE:
                printf("__");
                break;
            case FIELD_WALL:
                printf("XX");
                break;
            default:
                printf("??");
                break;
            }
        }
        printf("\n");
    }

    /* Makes us stronger: */
    map_heap_free(map_heap_container->prox);
    map_heap_container->prox = NULL;
    free(map_heap_container);
    map_heap_container = NULL;

    /* Party harder: */
    return 0;
}
