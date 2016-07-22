#include <stdio.h>

#include "hal.h"
#include "mock.h"

#define data_width 20
#define data_height 20

static FieldType types_empty[] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
typedef char check_empty_length[(sizeof(types_empty) / sizeof(types_empty[0])
    == (data_width * data_height)) ? 1 : -1];

static FieldType types_init[] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,2,2,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,2,0,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,0,1,2,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,2,2,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
typedef char check_types_length[(sizeof(types_init) / sizeof(types_init[0])
    == (data_width * data_height)) ? 1 : -1];

static FieldType types_patch[] = {
    1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
    0,2,2,1,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
    2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
typedef FieldType check_patch_length[(sizeof(types_patch) / sizeof(types_patch[0])
    == (MAP_PROXIMITY_SIZE * MAP_PROXIMITY_SIZE)) ? 1 : -1];

static FieldType types_after_update[] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
    0,0,0,0,1,2,2,1,0,0,0,0,0,0,0,0,0,0,1,0,
    0,2,2,2,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,2,0,2,2,2,1,2,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,1,0,1,2,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,2,2,0,0,0,1,1,0,
    0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
typedef char check_types_afterwards_length[(sizeof(types_after_update) == sizeof(types_init)) ? 1 : -1];

static FieldType types_patched_empty[] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
    0,0,0,0,0,2,2,1,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
    0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         /*|*/
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
typedef char check_empty_afterwards_length[(sizeof(types_patched_empty) == sizeof(types_init)) ? 1 : -1];

static void run_case(FieldType* input, FieldType* patch, FieldType* expected) {
    Map* m;
    Map* prox;

    /* Test setup */
    m = tests_load(input, data_width, data_height);
    prox = tests_load(patch, MAP_PROXIMITY_SIZE, MAP_PROXIMITY_SIZE);

    /* Run it. (Don't use this as a benchmark, x86 has caches and stuff
     * in contrast to PIC.) */
    map_merge(m, 4, 2, prox);

    tests_assert_equal(expected, m);

    map_heap_free(m);
    map_heap_free(prox);
}

int main(void) {
    run_case(types_empty, types_patch, types_patched_empty);
    run_case(types_init, types_patch, types_after_update);
    printf("\tDONE\n");
    return 0;
}
