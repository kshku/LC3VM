#include "lc3vm.h"

#include <assert.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
/* unix only */
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

// macros
#define MAX_MEMORY (1 << 16)

#define GET_BIT(x, n) (((x) >> (n)) & 1)

// typedefs
typedef enum Register {
    REGISTER_R0 = 0,
    REGISTER_R1,
    REGISTER_R2,
    REGISTER_R3,
    REGISTER_R4,
    REGISTER_R5,
    REGISTER_R6,
    REGISTER_R7,
    REGISTER_PC,
    REGISTER_COND,
    REGISTER_MAX
} Register;

typedef enum Opcode {
    OPCODE_BR = 0,
    OPCODE_ADD,
    OPCODE_LD,
    OPCODE_ST,
    OPCODE_JSR,
    OPCODE_AND,
    OPCODE_LDR,
    OPCODE_STR,
    OPCODE_RTI,
    OPCODE_NOT,
    OPCODE_LDI,
    OPCODE_STI,
    OPCODE_JMP,
    OPCODE_RES,
    OPCODE_LEA,
    OPCODE_TRAP,
    OPCODE_MAX
} Opcode;

typedef enum ConditionFlags {
    CONDFLAG_POS = 1 << 0,
    CONDFLAG_ZRO = 1 << 1,
    CONDFLAG_NEG = 1 << 2,
} ConditionFlags;

typedef enum TrapTable {
    TRAP_GETC =
        0x20, /* get character from keyboard, not echoed onto the terminal */
    TRAP_OUT = 0x21, /* output a character */
    TRAP_PUTS = 0x22, /* output a word string */
    TRAP_IN = 0x23, /* get character from keyboard, echoed onto the terminal */
    TRAP_PUTSP = 0x24, /* output a byte string */
    TRAP_HALT = 0x25 /* halt the program */
} TrapTable;

typedef enum MemoryMappedRegisters {
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02 /* keyboard data */
} MemoryMappedRegisters;

typedef void (*Operation)(uint16_t instruction);

// functions
static uint16_t memory_read(uint16_t address);

static uint16_t memory_write(uint16_t address, uint16_t value);

static void lc3vm_initialize(void);

static void lc3vm_shutdown(void);

static void disable_input_buffering();
static void restore_input_buffering();
static uint16_t check_key();
static void handle_interrupt(int signal);

static void lc3vm_br(uint16_t instruction);
static void lc3vm_add(uint16_t instruction);
static void lc3vm_ld(uint16_t instruction);
static void lc3vm_st(uint16_t instruction);
static void lc3vm_jsr(uint16_t instruction);
static void lc3vm_and(uint16_t instruction);
static void lc3vm_ldr(uint16_t instruction);
static void lc3vm_str(uint16_t instruction);
static void lc3vm_rti(uint16_t instruction);
static void lc3vm_not(uint16_t instruction);
static void lc3vm_ldi(uint16_t instruction);
static void lc3vm_sti(uint16_t instruction);
static void lc3vm_jmp(uint16_t instruction);
static void lc3vm_res(uint16_t instruction);
static void lc3vm_lea(uint16_t instruction);
static void lc3vm_trap(uint16_t instruction);

static uint16_t sext(uint16_t a, int bit_count);
// zext is not required

static void setcc(uint16_t reg);

static uint16_t swap16(uint16_t x);

// global or static vars
static bool running;

static uint16_t memory[MAX_MEMORY];

static uint16_t registers[REGISTER_MAX];

static Operation operations[OPCODE_MAX] = {
    [OPCODE_BR] = lc3vm_br,   [OPCODE_ADD] = lc3vm_add,
    [OPCODE_LD] = lc3vm_ld,   [OPCODE_ST] = lc3vm_st,
    [OPCODE_JSR] = lc3vm_jsr, [OPCODE_AND] = lc3vm_and,
    [OPCODE_LDR] = lc3vm_ldr, [OPCODE_STR] = lc3vm_str,
    [OPCODE_RTI] = lc3vm_rti, [OPCODE_NOT] = lc3vm_not,
    [OPCODE_LDI] = lc3vm_ldi, [OPCODE_STI] = lc3vm_sti,
    [OPCODE_JMP] = lc3vm_jmp, [OPCODE_RES] = lc3vm_res,
    [OPCODE_LEA] = lc3vm_lea, [OPCODE_TRAP] = lc3vm_trap,
};

// functin definitions
void lc3vm_run(void) {
    lc3vm_initialize();

    // printf("Memory:\n");
    // for (int i = 0x3000; i < 0x4000; ++i)
    //     printf("%u %u ", (memory[i] >> 8), (memory[i] & 0xff));
    // printf("\n");
    // running = false;

    // Run
    // int loop_count = 100;
    while (running) {
        // printf("registers[PC] = %#x\n", registers[REGISTER_PC]);
        uint16_t instruction = memory_read(registers[REGISTER_PC]++);
        // printf("instruction = %u %u\n", (instruction >> 8),
        //        (instruction & 0xff));
        uint16_t opcode = instruction >> 12;  // 4-bit opcode
        // printf("opcode = %u\n\n", (opcode & 0xff));
        operations[opcode](instruction);
        // if (!loop_count--) running = false;
    }

    lc3vm_shutdown();
}

void lc3vm_run_one_instr(void) {
    static bool initialized = false;
    if (!initialized) {
        lc3vm_initialize();
        initialized = true;
    }

    printf("\nregisters[PC] = %#x\n", registers[REGISTER_PC]);

    uint16_t instruction = memory_read(registers[REGISTER_PC]++);
    printf("params = %u %u\n", ((instruction >> 8) & 0xf),
           (instruction & 0xff));

    uint16_t opcode = instruction >> 12;  // 4-bit opcode
    printf("opcode = %u\n", opcode);

    operations[opcode](instruction);

    if (!running) lc3vm_shutdown();
}

bool read_image(const char *path) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;

    /* the origin tells us where in memory to place the image */
    uint16_t origin;
    fread(&origin, sizeof(origin), 1, file);
    origin = swap16(origin);

    /* we know the maximum file size so we only need one fread */
    uint16_t max_read = MAX_MEMORY - origin;
    uint16_t *p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    /* swap to little endian */
    while (read-- > 0) {
        *p = swap16(*p);
        ++p;
    }

    fclose(file);

    return true;
}

static uint16_t memory_read(uint16_t address) {
    if (address == MR_KBSR) {
        if (check_key()) {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = getchar();
        } else {
            memory[MR_KBSR] = 0;
        }
    }

    return memory[address];
}

static uint16_t memory_write(uint16_t address, uint16_t value) {
    memory[address] = value;
}

static void lc3vm_initialize(void) {
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    running = true;

    registers[REGISTER_COND] = CONDFLAG_ZRO;
    registers[REGISTER_PC] = 0x3000;  // default starting position
}

static void lc3vm_shutdown(void) {
    restore_input_buffering();
}

static void lc3vm_br(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_BR);

    if ((GET_BIT(instruction, 11) && (registers[REGISTER_COND] & CONDFLAG_NEG))
        || (GET_BIT(instruction, 10)
            && (registers[REGISTER_COND] & CONDFLAG_ZRO))
        || (GET_BIT(instruction, 9)
            && (registers[REGISTER_COND] & CONDFLAG_POS)))
        registers[REGISTER_PC] += sext(instruction & 0x1ff, 9);
}

static void lc3vm_add(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_ADD);

    uint16_t dest = (instruction >> 9) & 0x7;
    uint16_t param1 = (instruction >> 6) & 0x7;

    if (GET_BIT(instruction, 5))  // Immediate mode
        registers[dest] = registers[param1] + sext((instruction & 0x1f), 5);
    else registers[dest] = registers[param1] + registers[instruction & 0x7];

    setcc(dest);
}

static void lc3vm_ld(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_LD);

    uint16_t dest = (instruction >> 9) & 0x7;
    registers[dest] =
        memory_read(registers[REGISTER_PC] + sext(instruction & 0x1ff, 9));

    setcc(dest);
}

static void lc3vm_st(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_ST);

    memory_write(registers[REGISTER_PC] + sext(instruction & 0x1ff, 9),
                 registers[(instruction >> 9) & 0x7]);
}

static void lc3vm_jsr(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_JSR);

    registers[REGISTER_R7] = registers[REGISTER_PC];

    if (GET_BIT(instruction, 11))
        registers[REGISTER_PC] += sext(instruction & 0x7ff, 11);
    else registers[REGISTER_PC] = registers[(instruction >> 6) & 0x7];
}

static void lc3vm_and(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_AND);

    uint16_t dest = (instruction >> 9) & 0x7;
    uint16_t param1 = (instruction >> 6) & 0x7;

    if (GET_BIT(instruction, 5))  // Immediate mode
        registers[dest] = registers[param1] & sext((instruction & 0x1f), 5);
    else registers[dest] = registers[param1] & registers[instruction & 0x7];

    setcc(dest);
}

static void lc3vm_ldr(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_LDR);

    uint16_t dest = (instruction >> 9) & 0x7;
    registers[dest] = memory_read(registers[(instruction >> 6) & 0x7]
                                  + sext(instruction & 0x3f, 6));

    setcc(dest);
}

static void lc3vm_str(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_STR);

    memory_write(
        registers[(instruction >> 6) & 0x7] + sext(instruction & 0x3f, 6),
        registers[(instruction >> 9) & 0x7]);
}

static void lc3vm_rti(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_RTI);

    running = false;
}

static void lc3vm_not(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_NOT);

    uint16_t dest = (instruction >> 9) & 0x7;
    registers[dest] = ~registers[(instruction >> 6) & 0x7];
    setcc(dest);
}

static void lc3vm_ldi(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_LDI);

    uint16_t dest = (instruction >> 9) & 0x7;
    registers[dest] = memory_read(
        memory_read(registers[REGISTER_PC] + sext(instruction & 0x1ff, 9)));

    setcc(dest);
}

static void lc3vm_sti(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_STI);

    memory_write(
        memory_read(registers[REGISTER_PC] + sext(instruction & 0x1ff, 9)),
        registers[(instruction >> 9) & 0x7]);
}

static void lc3vm_jmp(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_JMP);

    registers[REGISTER_PC] = registers[(instruction >> 6) & 0x7];
}

static void lc3vm_res(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_RES);

    running = false;
}

static void lc3vm_lea(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_LEA);

    uint16_t dest = (instruction >> 9) & 0x7;
    registers[dest] = registers[REGISTER_PC] + sext(instruction & 0xff, 8);

    setcc(dest);
}

static void lc3vm_trap(uint16_t instruction) {
    assert(instruction >> 12 == OPCODE_TRAP);

    registers[REGISTER_R7] = registers[REGISTER_PC];
    // No need for zext
    // registers[REGISTER_PC] = memory_read(instruction & 0xff);

    switch (instruction & 0xFF) {
        case TRAP_GETC:
            registers[REGISTER_R0] = (uint16_t)getchar();
            setcc(REGISTER_R0);
            break;
        case TRAP_OUT:
            putc((char)registers[REGISTER_R0], stdout);
            fflush(stdout);
            break;
        case TRAP_PUTS: {
            /* one char per word */
            uint16_t *c = memory + registers[REGISTER_R0];
            while (*c) {
                putc((char)*c, stdout);
                ++c;
            }
            fflush(stdout);
        } break;
        case TRAP_IN: {
            printf("Enter a character: ");
            char c = getchar();
            putc(c, stdout);
            fflush(stdout);
            registers[REGISTER_R0] = (uint16_t)c;
            setcc(REGISTER_R0);
        } break;
        case TRAP_PUTSP: {
            /* one char per byte (two bytes per word)
               here we need to swap back to
               big endian format */
            uint16_t *c = memory + registers[REGISTER_R0];
            while (*c) {
                char char1 = (*c) & 0xFF;
                putc(char1, stdout);
                char char2 = (*c) >> 8;
                if (char2) putc(char2, stdout);
                ++c;
            }
            fflush(stdout);
        } break;
        case TRAP_HALT:
            puts("HALT");
            fflush(stdout);
            running = false;
            break;
        default:
            running = false;
    }
}

static uint16_t sext(uint16_t a, int bit_count) {
    if (GET_BIT(a, bit_count - 1)) a |= (0xffff << bit_count);
    return a;
}

static void setcc(uint16_t reg) {
    if (registers[reg] == 0) registers[REGISTER_COND] = CONDFLAG_ZRO;
    else if (registers[reg] >> 15) registers[REGISTER_COND] = CONDFLAG_NEG;
    else registers[REGISTER_COND] = CONDFLAG_POS;
}

static uint16_t swap16(uint16_t x) {
    return (x << 8) | (x >> 8);
}

struct termios original_tio;

void disable_input_buffering() {
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

uint16_t check_key() {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

void handle_interrupt(int signal) {
    restore_input_buffering();
    printf("\n");
    exit(-2);
}
