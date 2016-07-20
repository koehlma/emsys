#ifndef EPUCK_VICTIM_FINDER_H
#define EPUCK_VICTIM_FINDER_H

#include "sensors.h"

typedef struct VFLocals {
    double data[6];
} VFLocals;

typedef struct {
    VFLocals locals;
    unsigned int found_victim_xy;
    double victim_x;
    double victim_y;
} VFState;

void vf_reset(VFState* vf);
void vf_apply(double victim_angle, VFState* vf, Sensors* sens);

#endif /* EPUCK_VICTIM_FINDER_H */
