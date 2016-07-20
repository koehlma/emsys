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

static void filter_proximity_single(Sensors* sens, unsigned int index, double angle, double victim_phi, ExactPosition rel) {
    /* Is the center of the victim in the "cone of sight"? */
    if (fmod(fabs(sens->current.phi + angle - victim_phi), 2 * M_PI) < APERTURE_ANGLE) {
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

/* Do not pass 'victim' by reference. */
void filter_proximity(ExactPosition victim, Sensors* sens) {
    double victim_phi, dist;
    victim.x -= sens->current.x;
    victim.y -= sens->current.y;
    dist = sqrt(fabs(victim.x * victim.x) + fabs(victim.y * victim.y));

    if (dist > THRESHOLD_UPPER || dist < THRESHOLD_LOWER) {
        return;
    }

    victim_phi = atan2(victim.y, victim.x);
    filter_proximity_single(sens, PROXIMITY_P_20, 20*M_PI/180, victim_phi, victim);
    filter_proximity_single(sens, PROXIMITY_P_45, 45*M_PI/180, victim_phi, victim);
    filter_proximity_single(sens, PROXIMITY_P_90, 90*M_PI/180, victim_phi, victim);
    filter_proximity_single(sens, PROXIMITY_P_150, 150*M_PI/180, victim_phi, victim);
    filter_proximity_single(sens, PROXIMITY_M_150, -150*M_PI/180, victim_phi, victim);
    filter_proximity_single(sens, PROXIMITY_M_90, -90*M_PI/180, victim_phi, victim);
    filter_proximity_single(sens, PROXIMITY_M_45, -45*M_PI/180, victim_phi, victim);
    filter_proximity_single(sens, PROXIMITY_M_20, -20*M_PI/180, victim_phi, victim);
}
