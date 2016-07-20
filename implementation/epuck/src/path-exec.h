#ifndef PATH_EXEC_H
#define PATH_EXEC_H

#include "hal.h"
#include "map.h"
#include "sensors.h"

typedef struct PathExecInputs {
    unsigned int drive;
    unsigned int backwards;
    ExactPosition next;
} PathExecInputs;

typedef struct PathExecLocals {
    hal_time time_entered;
    double rotation_start_angle;
    double approx_rot_speed;
    double init_dir;
    ExactPosition start;
    double need_rot;
    double need_dist;
    ExactPosition normal;
    int state;
} PathExecLocals;

typedef struct PathExecState {
    PathExecLocals locals;
    unsigned int done;
    unsigned int see_obstacle;
} PathExecState;

void pe_reset(PathExecState* pe);
void pe_step(PathExecInputs* inputs, PathExecState* pe, Sensors* sens);

#endif
