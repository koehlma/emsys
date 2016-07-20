#ifndef EPUCK_VICTIM_FINDER_H
#define EPUCK_VICTIM_FINDER_H

#include "map.h"
#include "sensors.h"

typedef struct VFInputs {
    double x;
    double y;
    double phi;
} VFInputs;

typedef struct VFLocals {
    double data[6];
} VFLocals;

typedef struct {
    VFLocals locals;
    unsigned int found_victim_xy;
    ExactPosition victim;
} VFState;

void vf_reset(VFState* vf);
void vf_apply(VFInputs* inputs, VFState* vf);

#endif /* EPUCK_VICTIM_FINDER_H */
