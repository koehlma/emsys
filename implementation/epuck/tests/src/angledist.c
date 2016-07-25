#include <stdio.h>

#include "pi.h"

static int failures = 0;

static void assert_diff(double expected_deg, double dst_deg, double start_deg) {
    double actual_deg = angle_dist(dst_deg * M_PI / 180, start_deg * M_PI / 180) * 180.0 / M_PI;
    if (fabs(actual_deg - expected_deg) > 0.001) {
        printf("Failure for: assert(% 6.1f° == angle_diff(% 6.1f°, % 6.1f°)): was actually % 6.1f°\n",
            expected_deg, dst_deg, start_deg, actual_deg);
        ++failures;
    }
}

int main() {
    /* Check whether modulo works in both directions */
    assert_diff(0, 0, 0);
    assert_diff(0, 0, 360);
    assert_diff(0, 0, 720);
    assert_diff(0, 360, 0);
    assert_diff(0, 720, 0);

    /* Check whether "slightly off" works fine */
    assert_diff(-1, 0, 1);
    assert_diff(1, 1, 0);
    assert_diff(1, 0, 359);
    assert_diff(-1, 359, 0);

    /* Check whether "a bit off" works fine */
    assert_diff(2, -1, -3);
    assert_diff(2, 1, -1);
    assert_diff(2, 3, 1);
    assert_diff(-2, 1, 3);
    assert_diff(-2, -1, 1);
    assert_diff(-2, -3, -1);

    /* Check 'near 180' results */
    assert_diff(-179, 0, 179);
    assert_diff(179, 0, 181);
    assert_diff(179, 179, 0);
    assert_diff(-179, 181, 0);

    /* Can't check for exactly 180° things (non-determinism),
     * so don't do that. */

    /* Done. */
    printf("Done!\n");

    return failures;
}
