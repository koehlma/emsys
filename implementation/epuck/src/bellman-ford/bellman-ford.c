#include <limits.h>
#include <assert.h>
#include <stdio.h>
#include "bellman-ford.h"
#include "path-finder.h"

/* we have a strict upper limit for the maximal path size, therefore the following macro is
 * save and agrees with normal int comparison */
/* We need the - k for k  > 1 such that we do not get an overflow */
#define BF_INFINITY (INT_MAX - 5)

static BellmanFordLocals locals;

int pos2v(Position pos);
Position v2pos(int v);
static void init_bellman_ford(BellmanFord* state);
void reverse_path(Position* path, int path_length);

void find_path(BellmanFord* state) {
    int init = pos2v(state->init);
    int goal = pos2v(state->goal);
    int e_ix, p_ix, pred, p_len;
    const int weight = 1;
    int new_cost_u, new_cost_v, change;
    Edge current;

    init_bellman_ford(state);

    do {
        change = 0;
        for (e_ix = 0; e_ix < locals.edge_cnt; ++e_ix) {
            current = locals.edges[e_ix];

            /* update u if apropos */
            new_cost_u = locals.distances[current.v] + weight;
            if (new_cost_u < locals.distances[current.u]) {
                locals.distances[current.u] = new_cost_u;
                locals.pred[current.u] = current.v;
                change = 1;
            }
            /* update v if apropos */
            new_cost_v = locals.distances[current.u] + weight;
            if (new_cost_v < locals.distances[current.v]) {
                locals.distances[current.v] = new_cost_v;
                locals.pred[current.v] = current.u;
                change = 1;
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
        state->path[0] = INVALID_POS;
        assert(locals.distances[goal] == BF_INFINITY);
        p_len = p_ix = 0;
    }

    reverse_path(state->path, p_len);

    state->path[p_len] = INVALID_POS;

}

/* AUX */

void reverse_path(Position* path, int len) {
    int border = len/2; /* same border for odd AND even length */
    int i;
    Position tmp;
    for(i = 0; i < border; ++i){
        tmp = path[(len - i) - 1];
        path[(len - i) - 1] = path[i];
        path[i] = tmp;
    }
}

Position v2pos(int v) {
    Position res;
    assert(v < NUM_VERTICES);
    res.y = v / VERTICES_PER_ROW;
    res.y *= STEPPING_DIST;
    res.x = v % VERTICES_PER_COL;
    res.x *= STEPPING_DIST;
    assert(res.y < MAP_MAX_HEIGHT);
    return res;
}

int pos2v(Position pos) {
    int x = pos.x / STEPPING_DIST;
    int y = pos.y / STEPPING_DIST;
    int res = y * VERTICES_PER_ROW + x;
    assert(res < NUM_VERTICES);
    return res;
}

void init_bellman_ford(BellmanFord* state) {
    int i;
    int x_real, y_real, x_graph, y_graph, cheat, not_cheat, valid;
    Position curr, neighbour;
    int curr_v, neighbour_v, e_cnt;
    Edge new_edge;

    for(i = 0; i < NUM_VERTICES; ++i){
        locals.distances[i] = BF_INFINITY;
        locals.pred[i] = -1;
    }
    locals.distances[pos2v(state->init)] = 0;

    e_cnt = 0;
    for(x_graph = 0; x_graph < VERTICES_PER_ROW; ++x_graph){
        for(y_graph = 0; y_graph < VERTICES_PER_COL; ++y_graph){
            x_real = x_graph * STEPPING_DIST;
            y_real = y_graph * STEPPING_DIST;
            curr.x = x_real;
            curr.y = y_real;
            curr_v = pos2v(curr);

            /* NB: cheat is either 0 or 1, while not_cheat is the other. They represent the direction we are going. */
            for(cheat = 0; cheat < 2; ++cheat){
                not_cheat = 1 - cheat;
                neighbour.x = x_real + cheat * STEPPING_DIST;
                neighbour.y = y_real + not_cheat * STEPPING_DIST;

                valid = neighbour.x < MAP_MAX_WIDTH && neighbour.y < MAP_MAX_HEIGHT;
                if(valid && state->adj(curr, neighbour, state->map)) {
                    neighbour_v = pos2v(neighbour);
                    new_edge.u = curr_v;
                    new_edge.v = neighbour_v;
                    locals.edges[e_cnt] = new_edge;
                    ++e_cnt;
                }
            }
        }
    }
    locals.edge_cnt = e_cnt;
}
