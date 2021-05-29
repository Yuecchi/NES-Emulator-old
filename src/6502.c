#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "6502.h"

#define SET_FLAG(C, X) C->status_register |= (0x1 << X)
#define CLEAR_FLAG(C, X) C->status_register &= (0xff - (0x1 << X))
#define FLAG(C, X) C->status_register & (0x1 << X)

enum status_flag_t {
    CARRY,
    ZERO,
    INTERRUPT,
    DECIMAL,
    BREAK,
    OVERFLOW = 6,
    NEGATIVE
};

void set_status_flags(cpu_6502 *cpu, unsigned char reg) {
    if (reg & 0x80) {
        SET_FLAG(cpu, NEGATIVE);
        printf("negative status bit set\n");
    } else {
        CLEAR_FLAG(cpu ,NEGATIVE);
        printf("negative status bit cleared\n");
    }
    
    if (!reg) {
        SET_FLAG(cpu, ZERO);
        printf("zero status bit set\n");
    } else {
        CLEAR_FLAG(cpu, ZERO);
        printf("zero status bit cleared\n");
    }
}

void check_overflow(cpu_6502 *cpu, unsigned char reg, unsigned char operand) {
    if (reg + operand > 0xff) {
        SET_FLAG(cpu, CARRY);
    }
}

void check_underflow(cpu_6502 *cpu, unsigned char reg, unsigned char operand) {
    if (reg - operand < 0x0) {
        SET_FLAG(cpu, CARRY);
    }
}

/* 10 Branch on result plus
 * Branch on N = 0
 *
 * If the negative flag is clear, then increase
 * the value of the program counter by the operand
 */

void bpl_rel(cpu_6502 *cpu, operand_t *operand) {
    unsigned short jump_vector = cpu->program_counter + 2;
    if (!(FLAG(cpu, NEGATIVE))) {
        // must cast to char since jumps can be negative
        jump_vector += (char)operand->byte[0]; 
        printf("branching to 0x%04x\n", jump_vector);
    }
    cpu->program_counter = jump_vector;
}

/* 78 Set interrupt disable status
 * 1 -> I
 */
void sei_impl(cpu_6502 *cpu, operand_t *operand) {
    SET_FLAG(cpu, INTERRUPT);
    cpu->program_counter += 1;
    printf("interrupt status bit set\n");
}

/* 8D Store accumulator in memory (absolute)
 * A -> M
 */
void sta_abs(cpu_6502 *cpu, operand_t *operand) {
    mm_write(cpu->memory_map, operand->address, cpu->accumulator);
    cpu->program_counter += 3;
    printf("memory location 0x%04x set to 0x%02x\n", operand->address, mm_read(cpu->memory_map, operand->address));
}

/* 9A Transfer Index X to Stack Register
 *
 * X -> SP
 * 
 * Sets the value of the stack pointer to the
 * value held in register X
 */
void txs_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->stack_pointer = cpu->reg_x;
    printf("stack location set to 0x%02x\n", cpu->stack_pointer);
    cpu->program_counter += 1;
}

/* A0 Load index Y with memory (immediate)
 * M -> Y
 */
void ldy_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_y = operand->byte[0];
    printf("register Y set to 0x%02x\n", cpu->reg_y);
    set_status_flags(cpu, cpu->reg_y);
    cpu->program_counter += 2;
}

/* A2 Load index X with memory (immediate)
 * M -> X
 */
void ldx_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_x = operand->byte[0];
    printf("register X set to 0x%02x\n", cpu->reg_x);
    set_status_flags(cpu, cpu->reg_x);
    cpu->program_counter += 2;
}

/* A9 Load accumulator with memory (immediate)
 * M -> A
 */
void lda_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = operand->byte[0];
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;  
}

/* AD Load accumulator with memory (absolute)
 * M -> A
 */
void lda_abs(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = mm_read(cpu->memory_map, operand->address);
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 3;
}

/* BD Load accumulator with memory (absolute X)
 * M -> A
 */

void lda_absx(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = mm_read(cpu->memory_map, operand->address + cpu->reg_x);
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 3;
}

/* C9 Compare memory with accumulator (immediate)
 * A - M
 *
 * Compares the contents of the accumulator with an immediate 
 * value, and sets the zero, carry and negative flags as appropriate.
 * 
 * CARRY    : Set if A >= M
 * ZERO     : Set if A == M
 * NEGATIVE : Set if the result is negative 
 */

void cmp_imm(cpu_6502 *cpu, operand_t *operand) {
    unsigned char result = cpu->accumulator - operand->byte[0];
    printf("comparing 0x%02x to 0x%02x\n", cpu->accumulator, operand->byte[0]);
    set_status_flags(cpu, result);
    if (!(result & 0x80)) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }
}

/* D8 Clear decimal mode
 * 0 -> D
 */
void cld_impl(cpu_6502 *cpu, operand_t *operand) {
    CLEAR_FLAG(cpu, DECIMAL);
    cpu->program_counter += 1;
    printf("decimal status bit cleared\n");
}

operation_t instruction_set[0x100] = {
    /* 00 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 10 */    bpl_rel, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 20 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 30 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 40 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 50 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 60 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 70 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, sei_impl, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 80 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, sta_abs, NULL, NULL,
    /* 90 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, txs_impl, NULL, NULL, NULL, NULL, NULL,
    /* A0 */    ldy_imm, NULL, ldx_imm, NULL, NULL, NULL, NULL, NULL, NULL, lda_imm, NULL, NULL, NULL, lda_abs, NULL, NULL,
    /* B0 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, lda_absx, NULL, NULL,
    /* C0 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, cmp_imm, NULL, NULL, NULL, NULL, NULL, NULL,
    /* D0 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, cld_impl, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* E0 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* F0 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
};

cpu_6502 *create_6502(memory_map_t *memory_map) {
    cpu_6502 *cpu = calloc(1, sizeof(cpu_6502));
    cpu->memory_map = memory_map;
    cpu->program_counter = 0x10;
    return cpu;
}

void destroy_6502(cpu_6502 *cpu) {
    free(cpu);
}

void _6502_execute(cpu_6502 *cpu) {
    unsigned char opcode, operand[2];
    opcode     = mm_read(cpu->memory_map, cpu->program_counter);
    operand[0] = mm_read(cpu->memory_map, cpu->program_counter + 1);
    operand[1] = mm_read(cpu->memory_map, cpu->program_counter + 2);
    instruction_set[opcode](cpu, (operand_t*)&operand);
}

void _6502_set_pc(cpu_6502 * cpu, unsigned short address) {
    cpu->program_counter = address;
}
