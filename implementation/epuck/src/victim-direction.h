
#ifndef EPUCK_VICTIM_DIRECTION_H
#define EPUCK_VICTIM_DIRECTION_H

#include "sensors.h"
#include "t2t-data.h"

typedef struct VDLocals {
    hal_time time_begin;
    hal_time counter_total;
    hal_time counter_on;
    /* Here it is really important that we use double, not float. */
    double weighted_sum;
    /* Angle relative to starting angle */
    double gap_phi;
    int state;
} VDLocals;

typedef struct VDState {
    VDLocals locals;
    double victim_phi;
    unsigned int victim_found;
    unsigned int give_up;
    unsigned int digested; /* FIXME */
} VDState;

void vd_reset(VDState* vd);
void vd_step(T2TData_VicFix* input, VDState* vd, Sensors* sens);

#endif /* EPUCK_VICTIM_DIRECTION_H */
