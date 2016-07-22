#include <stdlib.h>
#include <stdio.h>

#include "hal.h"
#include "map.h"
#include "mock.h"
#include "prox-map.h"

static FieldType expected[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0,
    0, 2, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0,
    0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 2,
    0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0,
    0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 2,
    0, 2, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0,
};

int main() {
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
    printf("\tinitialized to low_left=(%d,%d)\n", pm.lower_left.x, pm.lower_left.y);

    /* Run it: */
    proximity_step(&pm, &sens);

    /* Check it: */
    tests_assert_equal(expected, map_get_proximity());

    /* Makes us stronger: */
    map_heap_free(map_heap_container->prox);
    map_heap_container->prox = NULL;
    free(map_heap_container);
    map_heap_container = NULL;

    /* Party harder: */
    return 0;
}
