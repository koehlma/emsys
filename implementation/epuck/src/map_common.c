#include <assert.h>
#include <stdint.h>
#include <stdlib.h> /* abs */
#include <string.h> /* memcpy */

#include "map.h"

/* ===== Common implementation of map.h ===== */

/* Check that constant in map-h */
typedef char check_neighborhood_map_length[(MAP_PROXIMITY_BUF_SIZE == MAP_INTERNAL_DATA_SIZE(MAP_PROXIMITY_SIZE,MAP_PROXIMITY_SIZE)) ? 1 : -1];

typedef unsigned long FieldPtr;

static FieldPtr locate(int x, int y, int w, int h) {
    FieldPtr bit_offset;

    assert(w >= 1 && h >= 1);
    /* I don't see why this should ever be violated */
    assert(w <= 256 && h <= 256);
    assert(x >= 0 && y >= 0 && x < w && y < h);
    bit_offset = (FieldPtr)(x + y * w);
    bit_offset *= BIT_PER_FIELD;
    return bit_offset;
}

/* Assume that a field always fits wholly into a byte: */
typedef char check_bit_length[((8 / BIT_PER_FIELD) * BIT_PER_FIELD == 8) ? 1 : -1];

static const unsigned int FIELD_MASK = (1 << BIT_PER_FIELD) - 1;

FieldType map_get_field(Map* map, int x, int y) {
    unsigned char* data;
    FieldPtr p;
    unsigned char raw_char;
    unsigned int bit_idx;
    FieldType result;
    data = map_serialize(map);
    p = locate(x, y, map_get_width(map), map_get_height(map));
    raw_char = data[p >> 3];
    bit_idx = p & 0x7; /* 0b111 */
    assert((bit_idx % BIT_PER_FIELD) == 0);
    result = (raw_char >> bit_idx) & FIELD_MASK;
    return result;
}

void map_set_field(Map* map, int x, int y, FieldType type) {
    unsigned char* data;
    FieldPtr p;
    unsigned char raw_char;
    unsigned int bit_idx;
    assert((type & ~FIELD_MASK) == 0);
    data = map_serialize(map);
    p = locate(x, y, map_get_width(map), map_get_height(map));
    raw_char = data[p >> 3];
    bit_idx = p & 0x7; /* 0b111 */
    assert((bit_idx % BIT_PER_FIELD) == 0);
    /* Clear old value: */
    raw_char &= ~(FIELD_MASK << bit_idx);
    /* Write new value: */
    raw_char |= type << bit_idx;
    /* Update array: */
    data[p >> 3] = raw_char;
}

void map_clear(Map* map) {
    memset(map_serialize(map), 0, (unsigned long)MAP_INTERNAL_DATA_SIZE(map_get_width(map),map_get_height(map)));
}

#define FIELDS_TO_BYTES(f) (((f)*BIT_PER_FIELD)/8)

void map_move(Map* map, int by_x, int by_y) {
    int w, h, row_bytes;
    unsigned char* data;

    /* == Initialization.  Nothing fancy. == */
    w = map_get_width(map);
    h = map_get_height(map);
    assert(by_x % (8 / BIT_PER_FIELD) == 0);
    assert(w % (8 / BIT_PER_FIELD) == 0);
    row_bytes = FIELDS_TO_BYTES(w);
    assert(row_bytes > 0);
    data = map_serialize(map);

    /* Do it in two separate steps, because that's easier,
     * and should be only marginally slower.
     * (And we don't actually care about performance here,
     * I just wanted this algorithm to be simple, and there's
     * not enough memory for a second buffer.) */

    /* == shift along X-axis == */
    assert(FIELD_UNKNOWN == 0);
    if (abs(by_x) >= w) {
        map_clear(map);
        /* Returning early is just an optimization, and does not change behavior. */
        return;
    } else if (by_x < 0) {
        int y;
        const int by_x_bytes = FIELDS_TO_BYTES(-by_x);
        assert(by_x_bytes > 0);
        assert(by_x_bytes < row_bytes);
        for (y = 0; y < h; ++y) {
            memmove(data + y * row_bytes,
                    data + by_x_bytes + y * row_bytes,
                    (unsigned int)(row_bytes - by_x_bytes));
            memset(data + (y + 1) * row_bytes - by_x_bytes, 0, (unsigned int)by_x_bytes);
        }
    } else if (by_x > 0) {
        int y;
        const int by_x_bytes = FIELDS_TO_BYTES(by_x);
        assert(by_x_bytes > 0);
        assert(by_x_bytes < row_bytes);
        for (y = 0; y < h; ++y) {
            memmove(data + by_x_bytes + y * row_bytes,
                    data + y * row_bytes,
                    (unsigned int)(row_bytes - by_x_bytes));
            memset(data + y * row_bytes,
                   0,
                   (unsigned int)by_x_bytes);
        }
    }

    /* == shift along Y-axis == */
    assert(FIELD_UNKNOWN == 0);
    if (abs(by_y) >= h) {
        map_clear(map);
    } else if (by_y < 0) {
        memmove(data,
                data + (-by_y) * row_bytes,
                (unsigned int)(row_bytes * (h - (-by_y))));
        memset(data + row_bytes * (h - (-by_y)),
               0,
               (unsigned int)(row_bytes * (-by_y)));
    } else if (by_y > 0) {
        memmove(data + by_y * row_bytes,
                data,
                (unsigned int)(row_bytes * (h - by_y)));
        memset(data,
               0,
               (unsigned int)(row_bytes * by_y));
    }
}

/* ===== map_merge algorithm (rest of file) ===== */

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
 *
 * DO NOT attempt to modify multiple bytes at once!
 * The PIC chips throw an exception on unaligned data access! */
typedef char check_map_merge_byte_bitlength[(BIT_PER_FIELD == 2) ? 1 : -1];
static char merge_byte(char previously, char input) {
    /* Create a mask of all 'fields' that "are not unknown",
     * which is equivalent to "are not 0b00". */
    char mask = input;
    mask |= (input & 0xAA) >> 1;
    mask |= (input & 0x55) << 1;
    /* Remove information that shall be overwritten. */
    previously &= ~mask;
    /* "or"-in all new bits. */
    return previously | input;
}

/* This implementation, too, is highly optimized.
 * Most importantly, it is optimized for the common case where the patch
 * fits wholly* in the map, and that would mean a lot of unnecessary branches.
 * (Each row is either 4 calls to merge_byte(), or 1 call to merge_long,
 * so naturally we hope for the latter.) */
void map_merge(Map* dst, int low_left_x, int low_left_y, Map* patch) {
    assert(low_left_x % 4 == 0);
    assert(map_get_width(patch) % 4 == 0);
    assert(map_get_width(patch) <= map_get_width(dst));
    assert(map_get_height(patch) <= map_get_height(dst));

    /* FIXME */
}
