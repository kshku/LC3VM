#include "lc3vm.h"

#include <assert.h>
#include <stdint.h>

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
    REGISTER_PSR,
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

typedef void (*Operation)(uint16_t instruction);

// functions
static uint16_t memory_read(uint16_t address);

static uint16_t memory_write(uint16_t address, uint16_t value);

static void lc3vm_initialize(void);

static void lc3vm_shutdown(void);

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

// global or static vars
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
void lc3_run(void) {
    lc3vm_initialize();

    // Run
    bool running = true;
    while (running) {
        uint16_t instruction = memory_read(registers[REGISTER_PC]++);
        uint16_t opcode = instruction >> 12;  // 4-bit opcode
        operations[opcode](instruction);
    }

    lc3vm_shutdown();
}

bool read_image(const char *file) {
    return false;
}

static uint16_t memory_read(uint16_t address) {
    return memory[address];
}

static uint16_t memory_write(uint16_t address, uint16_t value) {
    memory[address] = value;
}

static void lc3vm_initialize(void) {
    registers[REGISTER_COND] = CONDFLAG_ZRO;
    registers[REGISTER_PC] = 0x3000;  // default starting position
}

static void lc3vm_shutdown(void) {
    (void)(0);
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
    // TODO:
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

    // TODO:
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
    registers[REGISTER_PC] = memory_read(instruction & 0xff);
}

static uint16_t sext(uint16_t a, int bit_count) {
    if ((a >> bit_count) & 1) a |= (0xffff < bit_count);
    return a;
}

static void setcc(uint16_t reg) {
    if (reg == 0) registers[REGISTER_COND] = CONDFLAG_ZRO;
    else if (reg >> 15) registers[REGISTER_COND] = CONDFLAG_NEG;
    else registers[REGISTER_COND] = CONDFLAG_POS;
}
