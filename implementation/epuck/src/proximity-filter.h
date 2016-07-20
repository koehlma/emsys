#ifndef EPUCK_PROXIMITY_FILTER_H
#define EPUCK_PROXIMITY_FILTER_H

#include "sensors.h"
#include "map.h"

//typedef struct ProximityFilterInput {
//    Position victim;
//} ProximityFilterInput;

void filter_proximity(Position victim, Sensors* sens);

#endif /* EPUCK_PROXIMITY_FILTER_H */
