#include <stdlib.h>
#include "proximity-filter.h"
#include "sensors.h"

static const int THRESHOLD = 4;
static const double aperature_angle = 0.261799; /* 15 deg */

void filter_proximity(Position victim, Sensors* sens) {
    double delta_x = victim.x - sens->current.x;
    double delta_y = victim.y - sens->current.y;
    double dist = (int) (sqrt(fabs(delta_x * delta_x) + fabs(delta_y * delta_y)));
    double angle_tb_v;

    if(dist > THRESHOLD) {
        return;
    }

    angle_tb_v = atan2f(delta_y, delta_x);
    angle_tb_v = sens->current.phi - angle_tb_v;
    if(angle_tb_v < 0) {
        angle_tb_v += 2 * M_PI;
    }
    angle_tb_v -= M_PI;


}
