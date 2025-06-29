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

    lc3vm_run();

    // int run_type;
    // printf("Debug (1) or Run (2): ");
    // scanf("%d", &run_type);
    // getc(stdin); // ignore the enter pressed when giving input

    // if (run_type == 2) lc3vm_run();
    // else if (run_type != 1) return -1;

    // printf("Debug mode!\n");
    // printf("Press enter to run one step! or q to quit\n");

    // char ch = getc(stdin);
    // while (ch != 'q') {
    //     if (ch == '\n') lc3vm_run_one_instr();
    //     ch = getc(stdin);
    // }
}
