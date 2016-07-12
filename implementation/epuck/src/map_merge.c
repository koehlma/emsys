#include <stddef.h>
#include <stdint.h>

#include "hal.h" /* assert */
#include "map.h"

typedef char check_neighborhood_map_length[(MAP_PROXIMITY_BUF_SIZE == MAP_INTERNAL_DATA_SIZE(MAP_PROXIMITY_SIZE,MAP_PROXIMITY_SIZE)) ? 1 : -1];
typedef char check_max_is_valid[(0 < MAP_INTERNAL_DATA_SIZE(MAP_MAX_WIDTH,MAP_MAX_HEIGHT)) ? 1 : -1];

/* map_move and map_merge have to be hard-coded: */
typedef char check_bit_per_field[(BIT_PER_FIELD == 2) ? 1 : -1];

/* This implementation is highly optimized, but has lots of assumptions.
 * So I'll just go forward and assume BIT_PER_FIELD = 2.
 * If you change BIT_PER_FIELD, you have to rewrite quite a bit.
 *
 * Also, don't worry: in contrast to LuckySergionattarellaDeuce,
 * I'm intent on getting it perfectly right.
 * [LSD] https://github.com/RemyPorter/TDWTF-BYOC-Casino/tree/master/Week1/ben#all-about-luckysergiomattarelladeuce
 *
 * This function should take 9 instructions.
 * FIXME: Can someone verify that with a disassembler?
 * FIXME: Inline flags?  ('inline' is not C90, so need some clever matlab-checking)
 *
 * The PIC chips throw an exception on unaligned data access!
 * The reads and writes (done by the caller) *must* be properly aligned! */
typedef char check_map_merge_byte_bitlength[(BIT_PER_FIELD == 2) ? 1 : -1];
static uint16_t merge_u16(uint16_t previously, uint16_t input) {
    /* Create a mask of all 'fields' that "are not unknown",
     * which is equivalent to "are not 0b00". */
    uint16_t mask = input;
    mask |= (input & 0xAAAA) >> 1;
    mask |= (input & 0x5555) << 1;
    /* Remove information that shall be overwritten. */
    previously &= ~mask;
    /* "or"-in all new bits. */
    return previously | input;
}

/* This implementation, too, is highly optimized.
 * Hard requirements (if any of these is not met, then this doesn't work):
 * - the lower-left corner of the patch is in a "nice" spot (assert)
 *      -> guaranteed by prox_map.c
 * - the patch must fit *wholly* in the map (assert)
 *      -> guaranteed by prox_map.c (t2t.c should assert this, too)
 * - the patch must be a proximity map (assert)
 * - the internal map must be 2-byte aligned (attempted assert)
 *      -> guaranteed by map_stack.c, map_heap.c
 * - the patch (incoming packet buffer) must be 2-byte aligned (attempted assert)
 *      -> guaranteed by tinpuck/com.c
 *      -> guaranteed by hal/hal_matlab.c
 *      -> guaranteed by tests/mock.c
 * - BIT_PER_FIELD==2 (previous typedef-checks in the file) */
void map_merge(Map* dst, int low_left_x, int low_left_y, Map* patch) {
    int y, dst_two_rows_bytes;
    uint16_t* dst_data;
    uint16_t* patch_data;

    /* == Assert all assumptions == */
    assert(map_get_width(patch) % 4 == 0);
    /* Lower-left in "nice spot" */
    assert(low_left_x % 4 == 0);
    assert(low_left_y % 2 == 0);
    /* Fit wholly in map: */
    assert(map_get_width(patch) <= map_get_width(dst));
    assert(map_get_height(patch) <= map_get_height(dst));
    assert(low_left_x >= 0);
    assert(low_left_y >= 0);
    assert(low_left_x + map_get_width(patch) <= map_get_width(dst));
    assert(low_left_y + map_get_height(patch) <= map_get_height(dst));
    /* Is proximity map: */
    assert(map_get_width(patch) == MAP_PROXIMITY_SIZE);
    assert(map_get_height(patch) == MAP_PROXIMITY_SIZE);
    /* "Alignedness": */
    assert((((ptrdiff_t)(map_serialize(patch))) & 1) == 0);
    assert((((ptrdiff_t)(map_serialize(dst))) & 1) == 0);

    /* == Initialize == */
    #define PROX_TWO_ROWS_BYTES (MAP_PROXIMITY_SIZE/2)
    assert(PROX_TWO_ROWS_BYTES % 2 == 0);
    assert(MAP_PROXIMITY_SIZE * PROX_TWO_ROWS_BYTES / 2 == MAP_PROXIMITY_BUF_SIZE);
    dst_two_rows_bytes = map_get_width(dst) / 2;
    assert(dst_two_rows_bytes % 2 == 0);
    assert(map_get_height(dst) * dst_two_rows_bytes / 2 == MAP_INTERNAL_DATA_SIZE(map_get_width(dst),map_get_height(dst)));

    /* == Finally, the actual code == */
    /* We establish a view where the map consists of "blocks" that are precisely
     * 4 fields wide and 2 fields high, and fit precisely into a uin16_t.
     * In this view, *_data behave like "normally" indexed maps, i.e., x+y*w */
    #define PROX_CELL_TWO_ROWS_U16S (PROX_TWO_ROWS_BYTES/2)
    assert(4 == PROX_CELL_TWO_ROWS_U16S);
    dst_two_rows_bytes /= 2;
    /* Old name is now misleading, so us a macro instead: */
    #define DST_CELL_TWO_ROWS_U16S dst_two_rows_bytes
    low_left_x /= 4;
    low_left_y /= 2;
    dst_data = (uint16_t*)map_serialize(dst); /* BLESSED CAST */
    patch_data = (uint16_t*)map_serialize(patch); /* BLESSED CAST */
    dst_data += low_left_x + low_left_y * DST_CELL_TWO_ROWS_U16S;
    for (y = 0; y < MAP_PROXIMITY_SIZE / 2; ++y) {
        /* Manually unroll inner loop because I expect xc16 to be
         * too dumb for that. */
        dst_data[0 + y * DST_CELL_TWO_ROWS_U16S] = merge_u16(
            dst_data[0 + y * DST_CELL_TWO_ROWS_U16S], patch_data[0 + y * PROX_CELL_TWO_ROWS_U16S]);
        dst_data[1 + y * DST_CELL_TWO_ROWS_U16S] = merge_u16(
            dst_data[1 + y * DST_CELL_TWO_ROWS_U16S], patch_data[1 + y * PROX_CELL_TWO_ROWS_U16S]);
        dst_data[2 + y * DST_CELL_TWO_ROWS_U16S] = merge_u16(
            dst_data[2 + y * DST_CELL_TWO_ROWS_U16S], patch_data[2 + y * PROX_CELL_TWO_ROWS_U16S]);
        dst_data[3 + y * DST_CELL_TWO_ROWS_U16S] = merge_u16(
            dst_data[3 + y * DST_CELL_TWO_ROWS_U16S], patch_data[3 + y * PROX_CELL_TWO_ROWS_U16S]);
    }
}