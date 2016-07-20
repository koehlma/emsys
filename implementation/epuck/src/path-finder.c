#include <math.h>
#include <bellman-ford/bellman-ford.h>
#include <hal/map_heap.h>
#include <stdlib.h>

#include "hal.h"
#include "path-finder.h"
#include "pi.h"
#include "map.h"

enum {
    PF_inactive,
    PF_running,
    PF_complete
};

static int end_of_path_p(Position pos);
static int invalid_pos(Position pos, Map *map);

static Position map_discretize(double x, double y) {
    Position res;
    res.x = (int) (floor(x));
    res.y = (int) (floor(y));
    return res;
}

void pf_reset(PathFinderState* pf) {
    pf->locals.state = PF_inactive;
    pf->next.x = -1;
    pf->next.y = -1;
    pf->no_path = 0;
    pf->locals.path_index = 0;
    pf->locals.path[0].x = -1;
    pf->locals.path[0].y = -1;
    pf->path_completed = 0;
    pf->drive = 0;
    pf->backwards = 0;
}

static void pathing_failed(PathFinderState* pf) {
    pf->locals.state = PF_complete;
    pf->no_path = 1;
}


static void compute_path(PathFinderInputs* inputs, PathFinderState* pf, Sensors* sens) {
    Position dest, pos;
    int success;

    /* clear destination area */
    Map* map = map_get_accumulated();
    int delta_x, delta_y, x, y;
    for (delta_x = -3; delta_x < 4; delta_x++) {
        x = (int) inputs->dest_x + delta_x;
        if (x >= 0 && x < MAP_MAX_WIDTH ) {
            for (delta_y = -3; delta_y < 4; delta_y++) {
                y = (int) inputs->dest_y + delta_y;
                if (y >= 0 && y < MAP_MAX_HEIGHT) {
                    map_set_field(map, x, y, FIELD_FREE);
                }
            }

        }
    }

    pf->locals.state = PF_running;
    dest = map_discretize(inputs->dest_x, inputs->dest_y);
    pos = map_discretize(sens->current.x, sens->current.y);
    if (invalid_pos(pos, inputs->map)) {
        /* Uhh */
        pathing_failed(pf);
        return;
    }
    pf->locals.path_index = -1;
    success = pf_find_path(pos, dest, inputs->map, pf->locals.path);
    if (!success) {
        pathing_failed(pf);
    }
}

void pf_step(PathFinderInputs* inputs, PathFinderState* pf, Sensors* sens) {
    switch(pf->locals.state) {
        case PF_inactive:
            if (inputs->compute) {
               compute_path(inputs, pf, sens);
            }
            break;
        case PF_running:
            pf->drive = 1;
            assert(!inputs->step_complete || !inputs->step_see_obstacle);
            if (inputs->step_complete || pf->locals.path_index < 0) {
                Position next_wp;
                pf->drive = 0;
                pf->backwards = 0;
                /* Q: Why not check here for the position again?  Like this:
                 *    if(pos.x == pf->next.x && pos.y == pf->next.y)
                 * A: What do you if that fails?
                 *      (There's nothing meaningful one could do!)
                 *    Why would this happen anyway?
                 *      (PathExec already checks for stray!)
                 */
                pf->locals.path_index++;
                assert(pf->locals.path_index >= 0);
                next_wp = pf->locals.path[pf->locals.path_index];
                if (end_of_path_p(next_wp)) {
                    pf->locals.state = PF_complete;
                    pf->path_completed = 1;
                } else if (end_of_path_p(pf->locals.path[pf->locals.path_index + 1])) {
                    pf->backwards = inputs->is_victim;
                }
                pf->next.x = next_wp.x;
                pf->next.y = next_wp.y;
            }
            if (inputs->step_see_obstacle) {
                /* We ran into an obstacle -> internal map must be outdated. */
                compute_path(inputs, pf, sens);
                pf->drive = 0;
            }
            break;
        case PF_complete:
            /* Nothing to do here. */
            break;
        default:
            hal_print("PathFinder illegal state?!  Resetting ...");
            assert(0);
            pf_reset(pf);
            break;
    }
}

static int end_of_path_p(Position pos) {
    return pos.x == -1 && pos.y == -1;
}

static int adj(Position origin, Position goal, Map *map);

static int invalid_pos(Position pos, Map *map) {
    return pos.x >= map_get_width(map) || pos.x < 0 || pos.y >= map_get_height(map) || pos.y < 0;
}
static int occupied(Position *pos, Map *map) {
    return map_get_field(map, pos->x, pos->y) == FIELD_WALL;
}

static const int APPROX_RADIUS = 2;
static int adj(Position pos, Position goal, Map *map) {
    int i;
    Position test;
    int delta_x = (goal.x > pos.x) - (goal.x < pos.x);
    int delta_y = (goal.y > pos.y) - (goal.y < pos.y);
    int delta_x_orth = 1 - abs(delta_x);
    int delta_y_orth = 1 - abs(delta_y);

    assert(delta_x == 0 || delta_y == 0);
    assert(pos.x == goal.x || pos.y == goal.y);


    while(pos.x != goal.x || pos.y != goal.y) {
        if(abs(delta_x) == 1){
            test.x = pos.x;
            test.y = pos.y - APPROX_RADIUS;
        } else {
            test.x = pos.x - APPROX_RADIUS;
            test.y = pos.y;
        }
        for(i = 0; i < APPROX_RADIUS * 2 + 1; ++i) {
            if (!invalid_pos(test, map) && occupied(&test, map)) {
                return 0;
            }

            test.x += delta_x_orth;
            test.y += delta_y_orth;
        }
        pos.x += delta_x;
        pos.y += delta_y;
    }
    return 1;
}

int pf_find_path(Position position, Position goal, Map *map, Position *path) {
    BellmanFord state;
    state.adj = &adj;
    state.goal = goal;
    state.init = position;
    state.path = path;
    state.map = map;

    find_path(&state);

    if(state.path[0].x == -1 && state.path[0].y == -1) {
        path[0] = state.path[0];
        return 0;
    }

    return 1;
}
