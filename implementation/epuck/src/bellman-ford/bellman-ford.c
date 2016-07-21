#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "bellman-ford.h"
#include "hal.h"

#define LOG_BELLMAN_FORD

typedef char check_max_vertices_num[(NUM_VERTICES < 32767) ? 1 : -1];

static int16_t pos2v(ExactPosition pos);
static Position v2pos(int16_t v);
static unsigned int init_bellman_ford(BellmanFord* state);
static int generate_potential_neighbours(int16_t* buffer, int16_t v);

static unsigned int bellman_ford_cycle(BellmanFord* state) {
    int neigh_idx, neigh_num;
    int16_t neigh_buf[4];

    int16_t curr_v, neigh_v;
    unsigned int change = 0;

    change = 0;
    /* In each step, propagate our own "short" distance to our neighbors. */
    for (curr_v = 0; curr_v < NUM_VERTICES; ++curr_v) {
        if (state->distances_[curr_v] < 0) {
            /* "Infinitely far away; from holyday."
             * -- artist Somaaa, in track 'Soma Holiday' */
            continue;
        }
        neigh_num = generate_potential_neighbours(neigh_buf, curr_v);
        for (neigh_idx = 0; neigh_idx < neigh_num; ++neigh_idx) {
            neigh_v = neigh_buf[neigh_idx];
            if (!bf_adjacent_p(v2pos(curr_v), v2pos(neigh_v))) {
                continue;
            }

            /* Update neighbor if apropriate
             * (Overflow-aware code) */
            if (state->distances_[neigh_v] < 0
                || state->distances_[curr_v]
                   <= state->distances_[neigh_v] - 2) {
                /* "If we're at least two closer" \iff
                 * "If we are a shortcut, from our neighbor's point of view" */
                state->distances_[neigh_v] =
                    1 + state->distances_[curr_v];
                state->succ[neigh_v] = curr_v;
                change = 1;
            }
        }
    }

    return change;
}

#ifdef LOG_BELLMAN_FORD
static void print_path(BellmanFord* state) {
    char buf[50];
    int printed;
    int16_t i;

    hal_print("===BEGIN DUMP PATH===");
    sprintf(buf, "start=(%.1f,%.1f)", state->init.x, state->init.y);
    hal_print(buf);
    printed = 0;
    for (i = state->init_v; i >= 0; i = state->succ[i]) {
        ExactPosition pt;
        int printed_here;
        pt = bf_v2pos(state, i);
        if (printed > 50 - (1 + 6 + 1 + 6 + 2) - 1) {
            hal_print(buf);
            printed = 0;
        }
        printed_here = sprintf(buf + printed, "(%.1f,%.1f),",
            /* DO NOT CHANGE THE FORMAT SPECIFIER! (Unless you know
             * how to change the "printed > ..." expression below.) */
            pt.x, pt.y);
        if (printed_here < 0) {
            printed_here = sprintf(buf + printed, "?");
        }
        printed += printed_here;
    }
    if (printed != 0) {
        hal_print(buf);
    }
    sprintf(buf, "goal=(%.1f,%.1f)", state->goal.x, state->goal.y);
    hal_print(buf);
    hal_print("===END DUMP PATH===");
}
#endif

void find_path(BellmanFord* state) {
    if (!init_bellman_ford(state)) {
        state->init_v = -1;
        #ifdef LOG_BELLMAN_FORD
        hal_print("BF: Invalid start/end");
        #endif
        return;
    }

    while (bellman_ford_cycle(state))
        {}

    #ifdef LOG_BELLMAN_FORD
    print_path(state);
    #endif

    if (state->init_v != state->goal_v && state->succ[state->init_v] == -1) {
        /* There is no path.  (And no spoon.) */
        state->init_v = -1;
    } else {
        state->init_v = state->succ[state->init_v];
    }
}

/* AUX */

ExactPosition bf_v2pos(BellmanFord* state, int16_t v) {
    ExactPosition res;
    Position buf;

    if (v == state->goal_v) {
        return state->goal;
    }

    buf = v2pos(v);
    res.x = buf.x;
    res.y = buf.y;
    return res;
}

static Position v2pos(int16_t v) {
    Position res;
    assert(v < NUM_VERTICES);
    res.y = v / VERTICES_PER_ROW;
    res.y *= STEPPING_DIST;
    res.x = v % VERTICES_PER_COL;
    res.x *= STEPPING_DIST;
    assert(res.y < MAP_MAX_HEIGHT);
    return res;
}

static int16_t pos2v(ExactPosition pos) {
    int x, y, res;

    if (map_invalid_pos(map_discretize(pos))) {
        return -1;
    }

    x = (int)(floor(pos.x / STEPPING_DIST + 0.5));
    y = (int)(floor(pos.y / STEPPING_DIST + 0.5));
    res = y * VERTICES_PER_ROW + x;
    assert(0 <= res && res < NUM_VERTICES);
    return (int16_t)res;
}

static unsigned int init_bellman_ford(BellmanFord* state) {
    int i;
    state->init_v = pos2v(state->init);
    state->goal_v = pos2v(state->goal);
    if (state->init_v < 0 || state->init_v < 0) {
        return 0;
    }
    for(i = 0; i < NUM_VERTICES; ++i){
        state->distances_[i] = -1;
        state->succ[i] = -1;
    }
    state->distances_[state->goal_v] = 0;
    return 1;
}

static int generate_potential_neighbours(int16_t* buffer, int16_t v) {
    int i = 0;
    if (v % VERTICES_PER_ROW > 0) {
        buffer[i] = (int16_t)(v - 1);
        i += 1;
    }
    if (v % VERTICES_PER_ROW < VERTICES_PER_ROW - 1) {
        buffer[i] = (int16_t)(v + 1);
        i += 1;
    }
    if (v / VERTICES_PER_ROW > 0) {
        buffer[i] = (int16_t)(v - VERTICES_PER_ROW);
        i += 1;
    }
    if (v / VERTICES_PER_ROW < VERTICES_PER_COL - 1) {
        buffer[i] = (int16_t)(v + VERTICES_PER_ROW);
        i += 1;
    }
    return i;
}
