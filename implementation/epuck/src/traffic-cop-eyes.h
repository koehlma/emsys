#ifndef EPUCK_TRAFFIC_COP_EYES_H
#define EPUCK_TRAFFIC_COP_EYES_H

#include "sensors.h"

typedef struct TCEInputs {
    unsigned int found_victim_phi;
    unsigned int found_victim_xy;
    double ray_phi;
    unsigned int ir_stable;
    unsigned int phi_give_up;
} TCEInputs;

typedef struct TCELocals {
    double last_x;
    double last_y;
    double last_phi;
    hal_time time_start;
    int state;
} TCELocals;

typedef struct TCEState {
    TCELocals locals;
    unsigned int need_angle;
} TCEState;

void tce_reset(TCEState* tce);
void tce_step(TCEInputs* inputs, TCEState* tce, Sensors* sens);

#endif /* EPUCK_TRAFFIC_COP_EYES_H */
