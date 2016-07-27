#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "bellman-ford/bellman-ford.h"
#include "log_config.h"
#include "map.h"
#include "path-finder.h"
#include "pi.h"

enum {
    PF_inactive,
    PF_running,
    PF_complete
};

/* As soon as we're this far away from the victim, start driving backwards.
 * 2.65 victim radius (5.3/2)
 * 0.2  victim docking mechanism
 * 2.0  Tin Bot docking mechanism
 * 2.65 Tin Bot radius (5.3/2)
 * 0.5  magnet trigger distance */
#define PF_BACKWARDS_DIST (5.3+2.7)

/* If we aren't at least this close to the waypoint, restart. */
#define PE_TOL_HOME 8
#define PE_TOL_NORMAL 4
#define PE_TOL_VICTIM 2
#define PE_TOL_FIRST 3

void pf_reset(PathFinderState* pf) {
    pf->locals.state = PF_inactive;
    pf->next.x = -1;
    pf->next.y = -1;
    pf->no_path = 0;
    pf->locals.next_v = -1;
    pf->path_completed = 0;
    pf->drive = 0;
    pf->backwards = 0;
}

static void pathing_failed(PathFinderState* pf) {
    pf->locals.state = PF_complete;
    pf->locals.next_v = -1;
    pf->no_path = 1;
}

static void compute_path(PathFinderInputs* inputs, PathFinderState* pf, Sensors* sens) {
    int delta_x, delta_y, x, y;
    Map* map;
    ExactPosition pos;

    /* clear destination area */
    map = map_get_accumulated();
    for (delta_x = -3; delta_x < 4; delta_x++) {
        x = (int)( inputs->dest.x + delta_x );
        if (x >= 0 && x < MAP_MAX_WIDTH ) {
            for (delta_y = -3; delta_y < 4; delta_y++) {
                y = (int)( inputs->dest.y + delta_y );
                if (y >= 0 && y < MAP_MAX_HEIGHT) {
                    map_set_field(map, x, y, FIELD_FREE);
                }
            }
        }
    }

    pf->locals.state = PF_running;
    pf->locals.next_v = -2;
    pos.x = sens->current.x;
    pos.y = sens->current.y;
    pf_find_path(pos, inputs->dest, &pf->locals.bf_state);
    if (-1 == pf->locals.bf_state.init_v) {
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
            assert(pf->locals.next_v != -1);
            if (inputs->step_complete || pf->locals.next_v == -2) {
                int16_t last_v = pf->locals.next_v;
                #ifdef LOG_TRANSITIONS_PATH_FINDER
                hal_print("pf:feed next");
                #endif
                /* Need a full cycle of drive=0 */
                pf->drive = 0;
                pf->backwards = 0;
                /* Q: Why not check here for the position again?  Like this:
                 *    if(pos.x == pf->next.x && pos.y == pf->next.y)
                 * A: What do you if that fails?
                 *      (There's nothing meaningful one could do!)
                 *    Why would this happen anyway?
                 *      (PathExec already checks for stray!)
                 */
                pf->locals.next_v = pf->locals.bf_state.succ[pf->locals.next_v];
                assert(pf->locals.next_v >= -1);
                if (-1 == pf->locals.next_v) {
                    #ifdef LOG_TRANSITIONS_PATH_FINDER
                    hal_print("pf:... not. -> End!");
                    #endif
                    pf->locals.state = PF_complete;
                    pf->path_completed = 1;
                } else {
                    double dist;
                    pf->next = bf_v2pos(&pf->locals.bf_state, pf->locals.next_v);
                    #ifdef LOG_TRANSITIONS_PATH_FINDER
                    sprintf(hal_get_printbuf(), "pf: namely (%.1f,%.1f)",
                        pf->next.x, pf->next.y);
                    hal_print(hal_get_printbuf());
                    #endif
                    {
                        double tmp;
                        tmp = inputs->dest.x - pf->next.x;
                        dist = inputs->dest.y - pf->next.y;
                        dist = dist * dist + tmp * tmp;
                        dist = sqrt(dist);
                    }
                    if (dist < PF_BACKWARDS_DIST) {
                        if (inputs->is_victim) {
                            /* Docking to victim */
                            pf->drive_tol = PE_TOL_VICTIM;
                            pf->backwards = 1;
                        } else {
                            /* Home sweet home! */
                            pf->drive_tol = PE_TOL_HOME;
                        }
                    } else if (-2 == last_v) {
                        /* First waypoint */
                        pf->drive_tol = PE_TOL_FIRST;
                    } else {
                        /* Normal waypoint */
                        pf->drive_tol = PE_TOL_NORMAL;
                    }
                }
            }
            if (inputs->step_see_obstacle) {
                #ifdef LOG_TRANSITIONS_PATH_FINDER
                hal_print("pf:recompute");
                #endif
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

static int occupied(Position *pos, Map *map) {
    return map_get_field(map, pos->x, pos->y) == FIELD_WALL;
}

#define ADJ_RADIUS 4
static const double adj_weights[ADJ_RADIUS] = {
    1.0, 0.8, 0.5, 0.3
};
#define ADJ_THRESHOLD 2.5

unsigned int bf_adjacent_p(ExactPosition e_pos, ExactPosition e_goal) {
    int i;
    int delta_x, delta_y, delta_x_orth, delta_y_orth;
    Position test, pos, goal;
    Map* map;
    double accu_walls = 0;

    pos = map_discretize(e_pos);
    goal = map_discretize(e_goal);
    delta_x = (goal.x > pos.x) - (goal.x < pos.x);
    delta_y = (goal.y > pos.y) - (goal.y < pos.y);
    delta_x_orth = -delta_y;
    delta_y_orth = delta_x;

    assert(delta_x == 0 || delta_y == 0);
    assert(pos.x == goal.x || pos.y == goal.y);
    map = map_get_accumulated();

    while(pos.x != goal.x || pos.y != goal.y) {
        for(i = -ADJ_RADIUS; i <= ADJ_RADIUS; ++i) {
            test.x = pos.x + delta_x_orth * i;
            test.y = pos.y + delta_y_orth * i;
            if (map_invalid_pos(test) || occupied(&test, map)) {
                accu_walls += adj_weights[abs(i)];
                if (accu_walls > ADJ_THRESHOLD) {
                    return 0;
                }
            }
        }
        pos.x += delta_x;
        pos.y += delta_y;
    }
    return 1;
}

void pf_find_path(ExactPosition position, ExactPosition goal, BellmanFord* state) {
    state->goal = goal;
    state->init = position;

    #ifdef LOG_EXPENSIVE_PATH_FINDER
    {
        int x, y;
        Map* m;

        hal_print("PF:Begin map dump");
        m = map_get_accumulated();
        for (y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (x = 0; x < MAP_MAX_WIDTH; ++x) {
                switch (map_get_field(m, x, MAP_MAX_HEIGHT - y - 1)) {
                case FIELD_UNKNOWN:
                    hal_get_printbuf()[x] = ' ';
                    break;
                case FIELD_FREE:
                    hal_get_printbuf()[x] = '_';
                    break;
                case FIELD_WALL:
                    hal_get_printbuf()[x] = 'X';
                    break;
                case NUM_FIELD:
                    /* fall-through */
                default:
                    hal_get_printbuf()[x] = '?';
                    break;
                }
            }
            hal_get_printbuf()[MAP_MAX_WIDTH] = 0;
            hal_print(hal_get_printbuf());
        }
        hal_print("PF:End map dump");
    }
    #endif

    find_path(state);
}
