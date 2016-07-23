#ifndef EPUCK_PROXIMITY_FILTER_H
#define EPUCK_PROXIMITY_FILTER_H

#include "sensors.h"
#include "map.h"

void filter_prox_attached(Sensors* sens);
void filter_prox_detached(ExactPosition victim, Sensors* sens);

#endif /* EPUCK_PROXIMITY_FILTER_H */
