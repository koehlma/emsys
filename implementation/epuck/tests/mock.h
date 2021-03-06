#ifndef TESTS_MOCK_H
#define TESTS_MOCK_H

#include "hal.h"
#include "map_heap.h"

void tests_mock_tick(hal_time amount);

double tests_mock_get_debug(DebugCategory cat);

typedef struct ExpectPackage {
    char command;
    unsigned int length;
    unsigned char* data;
    int is_broadcast;
} ExpectPackage;

/* Can be called multiple times, even with static const things. */
void tests_mock_expect_next(const ExpectPackage* pkg);

void tests_mock_expect_set_enabled(int is_enabled);

void tests_mock_expect_assert_done(void);

Map* tests_load(FieldType* data, int width, int height);

void tests_assert_equal(FieldType* expect_map, Map* actual_map);

#endif
