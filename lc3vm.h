#pragma once

#include <stdbool.h>

bool read_image(const char *file);

void lc3vm_run(void);

void lc3vm_run_one_instr(void);
