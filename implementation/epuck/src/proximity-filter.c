#include <stdlib.h>
#include "proximity-filter.h"
#include "sensors.h"

#define THRESHOLD_UPPER 17
#define THRESHOLD_LOWER 5.3
#define APERTURE_ANGLE (15*M_PI/180) /* 15 deg */
#define VIC_RADIUS (5.3/2)

static double dist_from_ray(double angle, ExactPosition rel) {
    double dx, dy;

    dx = cos(angle);
    dy = sin(angle);
    return fabs(rel.x * dy - rel.y * dx);
}

static double dist_with_ray(double angle, ExactPosition rel) {
    double dx, dy;

    dx = cos(angle);
    dy = sin(angle);
    return rel.x * dx + rel.y * dy;
}

static void filter_proximity_single(Sensors* sens, unsigned int index, ExactPosition rel) {
    double victim_phi = atan2(rel.y, rel.x);
    double angle = prox_sensor_angle[index];

    /* Is the center of the victim in the "cone of sight"? */
    if (fmod(fabs(sens->current.phi + angle - victim_phi),
             2 * M_PI) < APERTURE_ANGLE) {
        sens->proximity[index] = 100;
        return;
    }

    /* Is the victim so close to our "cone of sight" that it's radius intersects
     * it?  (Even though the center isn't "visible"!)
     *     => that's the dist_from_ray calls
     * If so, is the victim really "in front" of us?
     *     => that's the dist_with_ray call */
    if (dist_with_ray(angle, rel) > 1
        && (dist_from_ray(angle - APERTURE_ANGLE, rel) < VIC_RADIUS
            || dist_from_ray(angle + APERTURE_ANGLE, rel) < VIC_RADIUS)) {
        /* Yes, it is. */
        sens->proximity[index] = 100;
    }
}

void filter_prox_attached(Sensors* sens) {
    sens->proximity[PROXIMITY_P_150] = 100;
    sens->proximity[PROXIMITY_M_150] = 100;
}

/* Do not pass 'victim' by reference. */
void filter_prox_detached(ExactPosition victim, Sensors* sens) {
    double dist;
    victim.x -= sens->current.x;
    victim.y -= sens->current.y;
    dist = sqrt(fabs(victim.x * victim.x) + fabs(victim.y * victim.y));

    if (dist > THRESHOLD_UPPER || dist < THRESHOLD_LOWER) {
        return;
    }

    filter_proximity_single(sens, PROXIMITY_P_20, victim);
    filter_proximity_single(sens, PROXIMITY_P_45, victim);
    filter_proximity_single(sens, PROXIMITY_P_90, victim);
    filter_proximity_single(sens, PROXIMITY_P_150, victim);
    filter_proximity_single(sens, PROXIMITY_M_150, victim);
    filter_proximity_single(sens, PROXIMITY_M_90, victim);
    filter_proximity_single(sens, PROXIMITY_M_45, victim);
    filter_proximity_single(sens, PROXIMITY_M_20, victim);
}
