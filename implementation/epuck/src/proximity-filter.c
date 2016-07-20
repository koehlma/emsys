#include <stdlib.h>
#include "proximity-filter.h"
#include "sensors.h"

#define THRESHOLD 17
#define APERATURE_ANGLE 15/180*M_PI /* 15 deg */

static void filter_proximity_single(Sensors* sens, unsigned int index, double angle, double victim_phi) {
    if (fabsf(fmodf(sens->current.phi + angle, 2*M_PI) - victim_phi) < APERATURE_ANGLE) {
        sens->proximity[index] = 100;
    }
}

void filter_proximity(Position victim, Sensors* sens) {
    double delta_x = victim.x - sens->current.x;
    double delta_y = victim.y - sens->current.y;
    double dist = (int) (sqrt(fabs(delta_x * delta_x) + fabs(delta_y * delta_y)));

    if (dist > THRESHOLD) {
        return;
    }

    double victim_phi = fmodf(atan2f(delta_y, delta_x) + 2*M_PI, 2*M_PI);
    filter_proximity_single(sens, PROXIMITY_P_20, 20/180*M_PI, victim_phi);
    filter_proximity_single(sens, PROXIMITY_P_45, 45/180*M_PI, victim_phi);
    filter_proximity_single(sens, PROXIMITY_P_90, 90/180*M_PI, victim_phi);
    filter_proximity_single(sens, PROXIMITY_P_150, 150/180*M_PI, victim_phi);
    filter_proximity_single(sens, PROXIMITY_M_150, (360-150)/180*M_PI, victim_phi);
    filter_proximity_single(sens, PROXIMITY_M_90, (360-90)/180*M_PI, victim_phi);
    filter_proximity_single(sens, PROXIMITY_M_45, (360-45)/180*M_PI, victim_phi);
    filter_proximity_single(sens, PROXIMITY_M_20, (360-20)/180*M_PI, victim_phi);

}