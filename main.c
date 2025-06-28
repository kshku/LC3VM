#include <stdio.h>

#include "lc3vm.h"

int main(int argc, const char *argv[]) {
    if (argc < 2) {
        /* show usage string */
        printf("lc3 [image-file1] ...\n");
        return 2;
    }

    for (int j = 1; j < argc; ++j) {
        if (!read_image(argv[j])) {
            printf("failed to load image: %s\n", argv[j]);
            return 1;
        }
    }
}
