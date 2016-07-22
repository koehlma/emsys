#include <stdio.h>

int main() {
    size_t b, i;
    float data[] = {0, 12.3f, 45.6f};
    for (i = 0; i < sizeof(data) / sizeof(data[0]); ++i) {
        union {float f; unsigned char ch[4];} conv;
        typedef char check_conv_length[(sizeof(conv) == sizeof(float)) ? 1 : -1];
        typedef char check_float_length[(sizeof(float) == 4) ? 1 : -1];
        (void)sizeof(check_conv_length);
        (void)sizeof(check_float_length);

        printf("%f = \n\t", data[i]);
        conv.f = data[i];
        for (b = 0; b < 4; ++b) {
            printf("0x%02x, ", conv.ch[b]);
        }
        printf("\n");
    }
    return 0;
}
