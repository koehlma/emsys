#include <stdio.h>

#include "hal.h"
#include "pi.h"
#include "victim-finder.h"

typedef struct Test {
    VFInputs m1;
    VFInputs m2;
    struct {double x; double y;} res;
} Test;

int main() {
    VFState vfs;
    Test the_test = {
        {45, 50, 0*M_PI/2}, {50, 45, 1*M_PI/2}, {50, 50}
    };

    vf_reset(&vfs);

    vf_apply(&the_test.m1, &vfs);
    assert(!vfs.found_victim_xy);

    vf_apply(&the_test.m2, &vfs);
    assert(vfs.found_victim_xy);

    printf("Actual %.1f,%.1f expected %.1f,%.1f\n",
        vfs.victim.x, vfs.victim.y, the_test.res.x, the_test.res.y);
    if (fabs(vfs.victim.x - the_test.res.x) < 0.01
        || fabs(vfs.victim.y - the_test.res.y) < 0.01) {
        printf("\t=> and that's GOOD!\n");
    } else {
        printf("\t=> and that's BAD!\n");
        return -1;
    }

    return 0;
}
