
#include "pi.h"

double angle_dist(double angle1, double angle2) {
    return fmod(angle1 - angle2 + M_PI, 2 * M_PI) - M_PI;
}