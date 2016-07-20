import enum
import io
import itertools

import numpy

import matplotlib as mpl
mpl.use('Agg')

from matplotlib import colors
from matplotlib import pyplot


# Keep the following definitions in sync with map.h.
# The names are identical in order to make the correlation
# unmistakeably clear.
MAP_PROX_SIZE = 16
MAP_MAX_WIDTH = 100
MAP_MAX_HEIGHT = 100


COLORMAP_1 = colors.ListedColormap(numpy.array([[1, 1, 1], [0.5, 0.5, 0.5]]))
COLORMAP_2 = colors.ListedColormap(numpy.array([[1, 1, 1], [0.5, 0.5, 0.5], [0, 0, 0]]))


# Possible values for a field
class Field(enum.IntEnum):
    UNKNOWN = 0
    FREE = 1
    WALL = 2


class Map:
    def __init__(self):
        self.array = numpy.zeros((100, 100), dtype=numpy.uint8)
        # access with: self.array[y][x]

    def update(self, x, y, field):
        if field != 0:
            self.array[y][x] = field

    def patch(self, ll_x, ll_y, data):
        # If the data is bad, don't enter it at all
        assert(0 <= ll_x and ll_x + MAP_PROX_SIZE < MAP_MAX_WIDTH)
        assert(0 <= ll_y and ll_y + MAP_PROX_SIZE < MAP_MAX_HEIGHT)
        assert(len(data) == MAP_PROX_SIZE * MAP_PROX_SIZE / 4)
        for x, y in itertools.product(
                range(int(MAP_PROX_SIZE / 4)), range(int(MAP_PROX_SIZE / 2))):
            # First, the byte of the "upper" four fields:
            val = data[int(0 + x + y * 2 * (MAP_PROX_SIZE / 4))]
            self.update(ll_x + x * 4 + 0, ll_y + y * 2 + 0, (val >> 0) & 0b11)
            self.update(ll_x + x * 4 + 1, ll_y + y * 2 + 0, (val >> 2) & 0b11)
            self.update(ll_x + x * 4 + 2, ll_y + y * 2 + 0, (val >> 4) & 0b11)
            self.update(ll_x + x * 4 + 3, ll_y + y * 2 + 0, (val >> 6) & 0b11)
            # Then, the byte of the "lower" four fields:
            val = data[int(1 + x + y * 2 * (MAP_PROX_SIZE / 4))]
            self.update(ll_x + x * 4 + 0, ll_y + y * 2 + 1, (val >> 0) & 0b11)
            self.update(ll_x + x * 4 + 1, ll_y + y * 2 + 1, (val >> 2) & 0b11)
            self.update(ll_x + x * 4 + 2, ll_y + y * 2 + 1, (val >> 4) & 0b11)
            self.update(ll_x + x * 4 + 3, ll_y + y * 2 + 1, (val >> 6) & 0b11)
        # Yeah, that's pretty convoluted.  However, it allows for several nice
        # optimizations in map_merge without exploding the alignment
        # properties that need to be obeyed by prox-map.c, function
        # 'desired_position'

    def get_image(self, close=True):
        buffer = io.BytesIO()
        figure = pyplot.figure()
        if numpy.max(m.array) == 2:
            color_map = COLORMAP_2
        else:
            color_map = COLORMAP_1
        pyplot.imshow(self.array, cmap=color_map, interpolation='nearest')
        figure.savefig(buffer, format='jpeg', dpi=100)
        pyplot.close(figure)
        return buffer.getvalue()


if __name__ == '__main__':
    from lps.commands import Commands
    x, y, *patch = Commands.T2T_UPDATE_MAP.decode(b'\x14\x00\x0e\x00\x00\x15\x05\x05\x00P\x00\x01UT\x15\x15TT\x01\x00PP\x15U\x15\x15\x00\x00EUUU\x05\x01\x00\x00TUUUUUUUUAUE\x01\x01U\x00PPAA\x05\x05\x00\x00TU\x00\x00\x15\x15\x00\x00')

    print(patch)

    m = Map()
    # print(m.get_image())
    m.patch(x, y, patch)
    with open('output.jpg', 'wb') as output:
        output.write(m.get_image())
    pyplot.show()

    print(m.array[y:y + 16, x:x + 16])
