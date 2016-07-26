#include <math.h>

#include "hal.h"
#include "prox-map.h"
#include "t2t-parse.h"
#include "sensors.h"
#include "state-machine-common.h"

/* Invariant: lower_left.x%4==0 && lower_left.y%2==0
 * This allows for an efficient implementation in map_merge and map_move. */

#define HALF_SIZE (MAP_PROXIMITY_SIZE * 0.5)
#define FLICKER_INERTIA 1
#define MIN_STEP_X 4
#define MIN_STEP_Y 2

#define MAX_MAP_PROXIMITY_DISTANCE 10
#define PROX_MAP_SEND_PERIOD 6

typedef char check_minstep_is_valid[(0 < MAP_INTERNAL_DATA_SIZE(MIN_STEP_X,MIN_STEP_Y)) ? 1 : -1];

static void desired_position(Position* into, Sensors* from) {
    into->x = MIN_STEP_X * (int)((from->current.x - HALF_SIZE) / MIN_STEP_X + 0.5);
    into->y = MIN_STEP_Y * (int)((from->current.y - HALF_SIZE) / MIN_STEP_Y + 0.5);
/* I hate doing this. */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
    into->x = MAX(into->x, 0);
    into->y = MAX(into->y, 0);
    into->x = MIN(into->x, MAP_MAX_WIDTH - MAP_PROXIMITY_SIZE);
    into->y = MIN(into->y, MAP_MAX_HEIGHT - MAP_PROXIMITY_SIZE);
#undef MIN
#undef MAX
    assert(into->x % MIN_STEP_X == 0 && into->y % MIN_STEP_Y == 0);
}

void proximity_reset(ProxMapState* prox_map, Sensors* sens) {
    desired_position(&prox_map->lower_left, sens);
    map_clear(map_get_proximity());
    prox_map->time_sent = hal_get_time();
}

static void maybe_move(ProxMapState* prox_map, Sensors* sens) {
    Position next;
    if (fabs(prox_map->lower_left.x + HALF_SIZE - sens->current.x) < 2 + FLICKER_INERTIA
        && fabs(prox_map->lower_left.y + HALF_SIZE - sens->current.y) < 1 + FLICKER_INERTIA) {
        return;
    }

    /* Need to move. */
    desired_position(&next, sens);
    /* If (for example) our offset increases, the map-data needs to shift
     * towards the negative.  Thus, we swap the signs here and let
     * map_move() believe that we are moving the map-data (and not our
     * frame of reference). */
    map_move(map_get_proximity(),
        next.x - prox_map->lower_left.x,
        next.y - prox_map->lower_left.y);

    prox_map->lower_left = next;
}

static void map_set_field_guarded(Map* map, int x, int y, FieldType type) {
    if (x >= 0 && x < MAP_PROXIMITY_SIZE && y >= 0 && y < MAP_PROXIMITY_SIZE) {
        map_set_field(map, x, y, type);
    }
}

static void enter_new_sensor_data(Map* map, Sensors* sens, double x, double y,
        unsigned int sensor_num) {
    double r = 0;
    double temp_x, temp_y;
    double distance = 5.3 / 2 + sens->proximity[sensor_num];
    double sensor_phi = prox_sensor_angle[sensor_num];
    double delta_x = cos(sens->current.phi + sensor_phi) * 0.7;
    double delta_y = sin(sens->current.phi + sensor_phi) * 0.7;
    x += delta_x;
    y += delta_y;
    while (r < distance && r < 7) {
        temp_x = x + delta_x;
        map_set_field_guarded(map, (int) temp_x, (int) y, FIELD_FREE);
        temp_y = y + delta_y;
        map_set_field_guarded(map, (int) x, (int) temp_y, FIELD_FREE);
        x = temp_x;
        y = temp_y;
        map_set_field_guarded(map, (int) x, (int) y, FIELD_FREE);
        r += 0.7;
    }
    x += 2 * delta_x;
    y += 2 * delta_y;
    r += 1.4;
    if (r >= distance && distance < MAX_MAP_PROXIMITY_DISTANCE) {
        map_set_field_guarded(map, (int) x, (int) y, FIELD_WALL);
    }
}

static void enter_new_data(ProxMapState* prox_map, Sensors* sens) {
    /* This will do all of the following:
     * - if a proximity sensor is on, add a wall (FIELD_WALL)
     * - if a proximity sensor is off, remove all walls for the next 7 cm (FIELD_FREE)
     * - except when there's VICTOR standing in the way.  (-> Proximity Filter)
     * - check whether x/y are legal coordinates with map_get_width(), map_get_height() */
    double rel_x = sens->current.x - prox_map->lower_left.x;
    double rel_y = sens->current.y - prox_map->lower_left.y;
    Map* map = map_get_proximity();
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_P_20);
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_P_45);
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_P_90);
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_P_150);
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_M_20);
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_M_45);
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_M_90);
    enter_new_sensor_data(map, sens, rel_x, rel_y, PROXIMITY_M_150);
}

static void proximity_send(ProxMapState* prox_map) {
    t2t_send_update_map(prox_map->lower_left.x, prox_map->lower_left.y,
        map_get_proximity());
    map_clear(map_get_proximity());
    prox_map->time_sent = hal_get_time();
}

void proximity_step(ProxMapState* prox_map, Sensors* sens) {
    maybe_move(prox_map, sens);
    enter_new_data(prox_map, sens);
    if (smc_time_passed_p(prox_map->time_sent, PROX_MAP_SEND_PERIOD)) {
        proximity_send(prox_map);
    }
}
