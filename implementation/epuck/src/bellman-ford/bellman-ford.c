#include <limits.h>
#include <assert.h>
#include "bellman-ford.h"
#include "path-finder.h"

/* we have a strict upper limit for the maximal path size, therefore the following macro is
 * save and agrees with normal int comparison */
/* We need the - k for k  > 1 such that we do not get an overflow */
#define BF_INFINITY (INT_MAX - 5)

static BellmanFordLocals locals;

static int pos2v(Position pos);
static Position v2pos(int v);
static void init_bellman_ford(BellmanFord* state);
static void reverse_path(Position* path, int path_length);
int generate_potential_neighbours(int* buffer, int v);

void find_path(BellmanFord* state) {
    int init = pos2v(state->init);
    int goal = pos2v(state->goal);
    int curr_v, p_ix, pred, p_len;
    const int weight = 1;
    int new_cost_curr, change;
    int neighbour_buffer[4];
    int num_neighbours, neighbour_ix;
    int neigh_v;

    init_bellman_ford(state);

    do {
        change = 0;
        for (curr_v = 0; curr_v < NUM_VERTICES; ++curr_v) {
            num_neighbours = generate_potential_neighbours(neighbour_buffer, curr_v);
            for(neighbour_ix = 0; neighbour_ix < num_neighbours; ++neighbour_ix) {
                neigh_v = neighbour_buffer[neighbour_ix];

                if(!state->adj(v2pos(curr_v), v2pos(neigh_v), state->map))
                    continue;

                /* update curr if apropos */
                new_cost_curr = locals.distances[neigh_v] + weight;
                if (new_cost_curr < locals.distances[curr_v]) {
                    locals.distances[curr_v] = new_cost_curr;
                    locals.pred[curr_v] = neigh_v;
                    change = 1;
                }
            }
        }
    } while (change);

    pred = goal;
    p_ix = 0;
    while(pred != -1 && pred != init) {
        state->path[p_ix] = v2pos(pred);
        pred = locals.pred[pred];
        ++p_ix;
    }
    p_len = p_ix;
    if(pred == -1) {
        state->path[0].x = -1;
        state->path[0].y = -1;
        assert(locals.distances[goal] == BF_INFINITY);
        p_len = p_ix = 0;
    }

    reverse_path(state->path, p_len);

    state->path[p_len].x = -1;
    state->path[p_len].y = -1;
}

/* AUX */

static void reverse_path(Position* path, int len) {
    int border = len/2; /* same border for odd AND even length */
    int i;
    Position tmp;
    for(i = 0; i < border; ++i){
        tmp = path[(len - i) - 1];
        path[(len - i) - 1] = path[i];
        path[i] = tmp;
    }
}

static Position v2pos(int v) {
    Position res;
    assert(v < NUM_VERTICES);
    res.y = v / VERTICES_PER_ROW;
    res.y *= STEPPING_DIST;
    res.x = v % VERTICES_PER_COL;
    res.x *= STEPPING_DIST;
    assert(res.y < MAP_MAX_HEIGHT);
    return res;
}

static int pos2v(Position pos) {
    int x = pos.x / STEPPING_DIST;
    int y = pos.y / STEPPING_DIST;
    int res = y * VERTICES_PER_ROW + x;
    assert(res < NUM_VERTICES);
    return res;
}

static void init_bellman_ford(BellmanFord* state) {
    int i;
    for(i = 0; i < NUM_VERTICES; ++i){
        locals.distances[i] = BF_INFINITY;
        locals.pred[i] = -1;
    }
    locals.distances[pos2v(state->init)] = 0;
}

int generate_potential_neighbours(int* buffer, int v) {
    int i = 0;
    if (v % VERTICES_PER_ROW > 0) {
        buffer[i] = v - 1;
        i += 1;
    }
    if (v % VERTICES_PER_ROW < VERTICES_PER_ROW - 1) {
        buffer[i] = v + 1;
        i += 1;
    }
    if (v / VERTICES_PER_ROW > 0) {
        buffer[i] = v - VERTICES_PER_ROW;
        i += 1;
    }
    if (v / VERTICES_PER_ROW < VERTICES_PER_COL - 1) {
        buffer[i] = v + VERTICES_PER_ROW;
        i += 1;
    }
    return i;
}
