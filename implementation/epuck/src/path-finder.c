#include <assert.h>
#include <math.h>

#include "astar/astar.h"

#include "path-finder.h"
#include "pi.h"

typedef struct Context {
	Map *map;
	Position goal;
} Context;

enum {
    PF_inactive,
    PF_running,
    PF_complete
};

extern const Position INVALID_POS; /* sigh. */
const Position INVALID_POS = {-1, -1};

static int eop(Position pos);

static Position map_discretize(Map* map, double x, double y) {
    Position res;
    res.x = (int) (floor(x / map->width));
    res.y = (int) (floor(y / map->height));
    return res;
}

void pf_reset(PathFinderState* pf) {
    pf->locals.state = PF_inactive;
    pf->next = INVALID_POS;
    pf->no_path = 0;
    pf->locals.path_index = 0;
    pf->locals.path[0] = INVALID_POS;
    pf->path_completed = 0;
}

static void compute_path(PathFinderInputs* inputs, PathFinderState* pf, Sensors* sens) {
    Position dest, pos, next_wp;
    pf->locals.state = PF_running;
    dest = map_discretize(pf->locals.map, inputs->dest_x, inputs->dest_y);
    pos = map_discretize(pf->locals.map, sens->current.x, sens->current.y);
    pf_find_path(pos, dest, pf->locals.map, pf->locals.path);
    pf->locals.path_index = 0;
    next_wp = pf->locals.path[pf->locals.path_index];
    if(eop(next_wp)) {
        pf->no_path = 1;
        pf->locals.state = PF_complete;
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
            assert(!inputs->step_complete || !inputs->step_see_obstacle);
            if (inputs->step_complete) {
                Position pos, next_wp;
                pos = map_discretize(pf->locals.map, sens->current.x, sens->current.y);
                next_wp = pf->locals.path[pf->locals.path_index];
                if (eop(next_wp)) {
                    pf->locals.state = PF_inactive;
                    pf->path_completed = 1;
                } else {
                    /* Q: Why not check here for the position again?  Like this:
                     *    if(pos.x == pf->next.x && pos.y == pf->next.y)
                     * A: What do you if that fails?
                     *      (There's nothing meaningful one could do!)
                     *    Why would this happen anyway?
                     *      (PathExec already checks for stray!)
                     */
                    pf->locals.path_index++;
                    next_wp = pf->locals.path[pf->locals.path_index];
                    if(eop(next_wp)) {
                        pf->locals.state = PF_complete;
                        pf->path_completed = 1;
                        pf->next = INVALID_POS;
                    }
                    pf->next = next_wp;
                }
            }
            if (inputs->step_see_obstacle) {
                /* We ran into an obstacle -> internal map must be outdated. */
                compute_path(inputs, pf, sens);
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

static int eop(Position pos) {
    return pos.x == INVALID_POS.x && pos.y == INVALID_POS.y;
}

static void find_neighbours(ASNeighborList neighbours, void *node, void *map);
static float heuristic(void *node, void *goal, void *map);
static int search_node_comparator(void *node1, void *node2, void *map);
static Position intersection(Position origin, Position goal, Map *map);

static int invalid_pos(Position pos, Map *map) {
    return pos.x >= map->width || pos.x < 0 || pos.y >= map->height || pos.y < 0;
}
static int occupied(Position *pos, Map *map) {
    return map->occupancy[pos->x][pos->y] != FIELD_WALL;
}

static const ASPathNodeSource path_node_source = {
        sizeof(Position),
        &find_neighbours,
        &heuristic,
        NULL,
        &search_node_comparator
};

void pf_find_path(Position position, Position goal, Map *map, Position *path) {
    ASPath  as_path;
    size_t path_length, i;
    Context context;

    context.map = map;
    context.goal = goal;
    as_path = ASPathCreate(&path_node_source, &context, &position, &goal);
    path_length = ASPathGetCount(as_path);

	for(i = 0; i < path_length; ++i) {
        assert(i < MAX_PATH_LENGTH);
		path[i] = *(Position *) ASPathGetNode(as_path, i);
	}
    assert(i <= MAX_PATH_LENGTH);
	path[i] = INVALID_POS;

	ASPathDestroy(as_path);
}

#define MAX_NEIGHBOURS 8

/**
 * For static upper bounds on the memory consumption we guarantee that the branching factor is no greater than 5
 */
static void find_neighbours(ASNeighborList neighbours, void *node, void *context_p) {
    double thetas[MAX_NEIGHBOURS];
    double cand_next_x, cand_next_y;
    double offset;
    int i;
    Position next, cand_next;

	Position *state = (Position *) node;
	Context *context = (Context *) context_p;


	if(heuristic(node, &context->goal, context->map) < STEP_DISTANCE){
		/* in reach of goal */
		Position special = intersection(*state, context->goal, context->map);
		ASNeighborListAdd(neighbours, &special, 1);
	}

    for(i = 0; i < MAX_NEIGHBOURS; ++i) {
        thetas[i] = rand() * (2 * M_PI * (1.0 / MAX_NEIGHBOURS));
    }

	for(i = 0; i < MAX_NEIGHBOURS; i++){
        offset = i * 2 * M_PI * (1.0 / MAX_NEIGHBOURS);
		cand_next_x = state->x + STEP_DISTANCE * cos(thetas[i] + offset);
		cand_next_y = state->y + STEP_DISTANCE * sin(thetas[i] + offset);
        cand_next.x = (int) (floor(cand_next_x));
        cand_next.y = (int) (floor(cand_next_y));
		next = intersection(*state, cand_next, context->map);
		ASNeighborListAdd(neighbours, &next, 1);
	}
}

static float heuristic(void *node, void *goal_node, void *context) {
    Position *state = (Position *) node;
    Position *goal = (Position *) goal_node;

	int diff_x = state->x - goal->x;
	int diff_y = state->y - goal->y;

    float h = (float) sqrt(diff_x * diff_x + diff_y * diff_y);
    (void)context; /* We don't actually use the context. */
	return h;
}

static int search_node_comparator(void *node1, void *node2, void *context) {
    Position p1 = *(Position *) node1;
    Position p2 = *(Position *) node2;

    int cmp_x = (p1.x > p2.x) - (p1.x < p2.x);
    int cmp_y = (p1.y > p2.y) - (p1.y < p2.y);
    (void)context; /* We don't actually use the context. */
	return cmp_x != 0 ? cmp_x : cmp_y;
}

static Position intersection(Position origin, Position goal, Map* map) {
	int delta_x = (goal.x > origin.x) - (goal.x < origin.x);
	int delta_y = (goal.y > origin.y) - (goal.y < origin.y);

	Position pos = origin;
	Position old = origin;
	while(pos.x != goal.x || pos.y != goal.y) {

		int x = pos.x, y = pos.y;
		if(abs(goal.x - pos.x) > abs(goal.y - pos.y))
			x += delta_x;
		else
			y += delta_y;

		pos.x = x;
        pos.y = y;
		if(invalid_pos(pos, map))
			return old;
		if(occupied(&pos, map))
			return old;

		old = pos;
	}
	return goal;
}
