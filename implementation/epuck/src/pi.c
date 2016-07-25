#include "pi.h"

double angle_dist(double dst, double start) {
    /* The obvious steps: */
    double dist = dst - start + M_PI;
    dist = fmod(dist, 2 * M_PI);
    /* In case 'dist' is negative: */
    dist = fmod(dist + 2 * M_PI, 2 * M_PI);
    return dist - M_PI;
}
