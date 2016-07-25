#ifndef PI_H
#define PI_H

#include <math.h> /* Probably doesn't define M_PI like it would in C99 */

#ifndef M_PI
/* Stolen from math.h ... yeah. */
#define M_PI 3.14159265358979323846 /* pi */
#endif

/* NB: The result might be negative. Use fabs if not needed. */
double angle_dist(double angle1, double angle2);

#endif
