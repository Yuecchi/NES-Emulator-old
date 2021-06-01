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

/*
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
*/

static void push(cpu_6502 *cpu, unsigned char byte) {
    unsigned short stack_location = cpu->stack_pointer + 0x100;
    printf("pushing 0x%02x to stack location 0x%04x\n", byte, stack_location);
    mm_write(cpu->memory_map, stack_location, byte);
    cpu->stack_pointer -= 1;
}

static unsigned char pop(cpu_6502 *cpu) {
    cpu->stack_pointer += 1;
    unsigned short stack_location = cpu->stack_pointer + 0x100;
    unsigned char byte = mm_read(cpu->memory_map, stack_location);
    printf("popping element 0x%02x from stack at location 0x%04x\n", byte, stack_location);  
    return byte;
}

/* function for determining if a page change has occurred
 *
 * return: 1 if a page change has occured, otherwise 0
 */
static unsigned int page_changed(unsigned int current_address, unsigned int target_address) {
    unsigned int current_page = current_address / 0x100;
    unsigned int target_page  = target_address  / 0x100;
    return current_page != target_page;
}

/* 09 OR memory with accumulator (immediate)
 * 
 * A OR M -> A
 * 
 * Sets the value of the accumulator equal to the result
 * of the value currently held by the accumulator ORed 
 * with the given operand.
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int ora_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator |= operand->byte[0];
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 2;
}

/* 10 Branch on result plus (relative)
 *
 * Branch on N = 0
 *
 * If the negative flag is clear, then change
 * the value of the program counter by the value
 * of the operand
 * 
 * Bytes:  2
 * Cycles: 2 if no branching operation occurs
 *         3 if the branching operation occurs
 *         4 if the branching operation occurs 
 *           and the destination is on a new page
 * 
 * note: A page boundry crossing occurs when the
 *       branch destination is on a different page 
 *       than the instruction AFTER the branch 
 *       instruction
 */

unsigned int bpl_rel(cpu_6502 *cpu, operand_t *operand) {
    unsigned short jump_vector = cpu->program_counter + 2;
    unsigned int cycles = 2;
    if (!(FLAG(cpu, NEGATIVE))) {
        // must cast to char since jumps can be negative
        jump_vector += (char)operand->byte[0];
        // add an extra cycle since the branch succeeded
        // and add an additional cycle if the program 
        // counter is moved to a new page
        cycles += (1 + page_changed(cpu->program_counter + 2, jump_vector));
        printf("branching to 0x%04x\n", jump_vector);
    } else {
        printf("negative flag was not clear, no action taken\n");
    }
    cpu->program_counter = jump_vector;
    return cycles;
}

/*
 * 20 Jump to subroutine (absolute)
 *
 * Sets the program counter to the address indicated
 * by the operand. Before the program counter is 
 * modified, the current value held by the program
 * counter is pushed onto the program stack, hi byte
 * first.
 * 
 * Bytes:  3
 * Cycles: 6
 */

unsigned int jsr_abs(cpu_6502 *cpu, operand_t *operand) {
    unsigned short jump_vector = operand->address;
    cpu->program_counter += 2;
    push(cpu, (unsigned char)(cpu->program_counter >> 8)); // push hi byte
    push(cpu, (unsigned char)(cpu->program_counter)); // push lo byte
    printf("jumping to subroutine at 0x%04x\n", jump_vector);
    cpu->program_counter = jump_vector;
    return 6;
}

/* 29 AND memory with accumulator (immediate)
 * 
 * A AND M -> A
 * 
 * Sets the value of the accumulator equal to the result
 * of the value currently held by the accumulator ANDed 
 * with the given operand.
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int and_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator &= operand->byte[0];
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 2;
}

/* 2C Tests bits in memory with accumulator (absolute)
 * 
 * Fetches a value from the location in memory given by
 * the operand. The negative flag and overflow are made
 * equal to bits 7 and 6 of the fetched value respectively.
 * The zero status flag is set if the result of ANDing the 
 * fetched value with the value held by the accumulator results
 * in zero.
 * 
 * e.g:
 * 
 * Z = !(A & M)
 * N = M7
 * V = M6
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int bits_abs(cpu_6502 *cpu, operand_t *operand) {
    // fetch value
    unsigned char value = mm_read(cpu->memory_map, operand->address); 
    // set the negative status flag equal to the
    // 7th bit of the fetched value
    if (value & 0x80) {
        SET_FLAG(cpu, NEGATIVE);
        printf("negative status bit set\n");
    } else {
        CLEAR_FLAG(cpu, NEGATIVE);
        printf("negative status bit cleared\n");
    }
    // set the overflow status flag equal to the
    // 6th bit of the fetched value
    if (value & 0x40) {
        SET_FLAG(cpu, OVERFLOW);
        printf("overflow status bit set\n");
    } else {
        CLEAR_FLAG(cpu, OVERFLOW);
        printf("overflow status bit cleared\n");
    }
    // set the zero status if the result of the fetched
    // value ANDed with the value held by the accumulator
    // is zero
    if (!(cpu->accumulator & value)) {
        SET_FLAG(cpu, ZERO);
        printf("zero status bit set\n");
    } else {
        CLEAR_FLAG(cpu, ZERO);
        printf("zero status bit cleared\n");
    }
    cpu->program_counter += 3;
    return 4;
}

/* 4C Jump to new location (absolute)
 *
 * operand -> PC
 * 
 * Bytes:  3
 * Cycles: 3
 */

unsigned int jmp_abs(cpu_6502 *cpu, operand_t *operand) {
    unsigned short jump_vector = operand->address;
    printf("jumping to 0x%04x\n", jump_vector);
    cpu->program_counter = jump_vector;
    return 3;
}

/* 60 Return from suboutine
 *
 * Returns from the last subroutine the program entered.
 * This is achieved by taking the top two elements on the
 * program stack and using them to set the program counter
 * such that the first element becomes the lo-byte of the
 * program counter and the next element becomes the hi-byte
 * of the program counter
 * 
 * e.g:
 * 
 * pc = pop() + (pop() * 0x100)
 * 
 * Bytes:  1
 * Cycles: 6
 */

unsigned int rts_impl(cpu_6502 *cpu, operand_t *operand) {
    unsigned short jump_vector = pop(cpu) + (pop(cpu) * 0x100);
    printf("returning from subroutine to 0x%04x\n", jump_vector + 1);
    cpu->program_counter = jump_vector + 1;
    return 6;
}

/* 78 Set interrupt disable status
 * 
 * 1 -> I
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int sei_impl(cpu_6502 *cpu, operand_t *operand) {
    SET_FLAG(cpu, INTERRUPT);
    cpu->program_counter += 1;
    printf("interrupt status bit set\n");
    return 2;
}


/* 85 Store accumulator in memory (zeropage)
 *
 * A -> M
 * 
 * Stores the value held by the accumlator at the 
 * location in memory indicated by the operand.
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int sta_zpg(cpu_6502 *cpu, operand_t *operand) {
    mm_write(cpu->memory_map, operand->byte[0], cpu->accumulator);
    printf("memory location 0x%04x set to 0x%02x\n", operand->byte[0], mm_read(cpu->memory_map, operand->byte[0]));
    cpu->program_counter += 2;
    return 3;
}

/* 86 Store index X in memory (zeropage)
 *
 * X -> M
 * 
 * Stores the value held by the X register at the 
 * location in memory indicated by the operand.
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int stx_zpg(cpu_6502 *cpu, operand_t *operand) {
    mm_write(cpu->memory_map, operand->byte[0], cpu->reg_x);
    printf("memory location 0x%04x set to 0x%02x\n", operand->byte[0], mm_read(cpu->memory_map, operand->byte[0]));
    cpu->program_counter += 2;
    return 3;
}

/* 88 Decrement index Y by one (implied)
 *
 * Y - 1 -> Y
 * 
 * Decrements the value held by register Y by one
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int dey_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_y -= 1;
    printf("register Y reduced to 0x%02x\n", cpu->reg_y);
    set_status_flags(cpu, cpu->reg_y); 
    cpu->program_counter += 1;
    return 2;
}

/* 8A transfer index X to accumulator (implied)
 *
 * X -> A
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int txa_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = cpu->reg_x;
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->reg_y);
    cpu->program_counter += 1;
    return 2;
}

/* 8D Store accumulator in memory (absolute)
 *
 * A -> M
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int sta_abs(cpu_6502 *cpu, operand_t *operand) {
    mm_write(cpu->memory_map, operand->address, cpu->accumulator);
    printf("memory location 0x%04x set to 0x%02x\n", operand->address, mm_read(cpu->memory_map, operand->address));
    cpu->program_counter += 3;
    return 4;
}

/* 91 Store accumulator in memory (indirect y)
 *
 * A -> M
 * 
 * Stores the value held by the accumulator at a target
 * address which is fetched from memory. The location of
 * the lo-byte of the target address is given by the operand,
 * and the location of the hi-byte of the target addres is 
 * given by the operand + 1.
 * 
 * e.g:
 * 
 * | operand | operand + 1 | 
 * | LO-BYTE |   HI-BYTE   |
 * 
 * target_addr = (HI-BYTE * 0x100) + LO-BYTE
 * 
 * Upon fetching the address, the final target address
 * location is then given by adding the value of the Y
 * register to the fetched address
 * 
 * e.g:
 * 
 * final_addr = target_addr + Y
 * 
 * The value of the accumulator is then stored at the final
 * address.
 * 
 * Bytes:  2
 * Cycles: 6
 */

unsigned int sta_indy(cpu_6502 * cpu, operand_t *operand) {
    unsigned short target_address = (mm_read(cpu->memory_map, operand->byte[0] + 1) << 8) + mm_read(cpu->memory_map, operand->byte[0]) + cpu->reg_y;
    mm_write(cpu->memory_map, target_address, cpu->accumulator);
    printf("memory location 0x%04x set to 0x%02x\n", target_address, mm_read(cpu->memory_map, target_address));
    cpu->program_counter += 2;
    return 6;
}

/* 99 Store accumulator in memory (absolute Y)
 *
 * A -> M
 * 
 * Bytes:  3
 * Cycles: 5
 */

unsigned int sta_absy(cpu_6502 *cpu, operand_t *operand) {
    unsigned short target_address = operand->address + cpu->reg_y;
    mm_write(cpu->memory_map, target_address, cpu->accumulator);
    printf("memory location 0x%04x set to 0x%02x\n", target_address, mm_read(cpu->memory_map, target_address));
    cpu->program_counter += 3;
    return 5;
}

/* 9A Transfer Index X to Stack Register (implied)
 *
 * X -> SP
 * 
 * Sets the value of the stack pointer to the
 * value held in register X
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int txs_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->stack_pointer = cpu->reg_x;
    printf("stack location set to 0x%02x\n", cpu->stack_pointer);
    cpu->program_counter += 1;
    return 2;
}

/* A0 Load index Y with memory (immediate)
 *
 * M -> Y
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int ldy_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_y = operand->byte[0];
    printf("register Y set to 0x%02x\n", cpu->reg_y);
    set_status_flags(cpu, cpu->reg_y);
    cpu->program_counter += 2;
    return 2;
}

/* A2 Load index X with memory (immediate)
 *
 * M -> X
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int ldx_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_x = operand->byte[0];
    printf("register X set to 0x%02x\n", cpu->reg_x);
    set_status_flags(cpu, cpu->reg_x);
    cpu->program_counter += 2;
    return 2;
}

/* A9 Load accumulator with memory (immediate)
 *
 * M -> A
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int lda_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = operand->byte[0];
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 2;
}

/* AD Load accumulator with memory (absolute)
 *
 * M -> A
 * 
 * Fetches a value from the location in memory given
 * by the operand and stores the fetched value in the
 * accumulator
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int lda_abs(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = mm_read(cpu->memory_map, operand->address);
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 3;
    return 4;
}

/* B0 Branch on carry set (relative)
 *
 * Branch on C = 1
 *
 * If the carry flag is set, then change
 * the value of the program counter by the 
 * value of the operand
 * 
 * Bytes:  2
 * Cycles: 2 if no branching operation occurs
 *         3 if the branching operation occurs
 *         4 if the branching operation occurs 
 *           and the destination is on a new page
 * 
 * note: A page boundry crossing occurs when the
 *       branch destination is on a different page 
 *       than the instruction AFTER the branch 
 *       instruction
 */

unsigned int bcs_rel(cpu_6502 *cpu, operand_t *operand) {
    unsigned short jump_vector = cpu->program_counter + 2;
    unsigned int cycles = 2;
    if ((FLAG(cpu, CARRY))) {
        // must cast to char since jumps can be negative
        jump_vector += (char)operand->byte[0]; 
        // add an extra cycle since the branch succeeded
        // and add an additional cycle if the program 
        // counter is moved to a new page
        cycles += (1 + page_changed(cpu->program_counter + 2, jump_vector));
        printf("branching to 0x%04x\n", jump_vector);
    } else {
        printf("carry flag was clear, no action taken\n");
    }
    cpu->program_counter = jump_vector;
    return cycles;
}

/* BD Load accumulator with memory (absolute X)
 *
 * M -> A
 * 
 * Bytes:  3
 * Cycles: 4 if no page boundry is crossed
 *         5 if a page boundry is crossed
 */

unsigned int lda_absx(cpu_6502 *cpu, operand_t *operand) {
    unsigned int cycles = 4;
    unsigned short target_address = operand->address + cpu->reg_x;
    cpu->accumulator = mm_read(cpu->memory_map, target_address);
    // add an additional cycle if the base address
    // is on a different page to the address plus
    // the offset
    cycles += page_crossed(operand->address, target_address);
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 3;
    return cycles;
}

/* C0 Compare memory with index Y (immediate)
 *
 * Y - M
 *
 * Compares the contents of the Y register with an immediate 
 * value, and sets the zero, carry and negative flags as appropriate.
 * 
 * CARRY    : Set if Y >= M (unsigned comparison)
 * ZERO     : Set if Y == M
 * NEGATIVE : Set if the result is negative
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int cpy_imm(cpu_6502 *cpu, operand_t *operand) {
    unsigned char result = cpu->reg_y - operand->byte[0];
    printf("comparing 0x%02x to 0x%02x\n", cpu->reg_y, operand->byte[0]);
    set_status_flags(cpu, result);
    if (cpu->reg_y >= operand->byte[0]) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }
    cpu->program_counter += 2;
    return 2;
}

/* C8 Increment index Y by one
 *
 * Y + 1 -> Y
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int iny_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_y += 1;
    set_status_flags(cpu, cpu->reg_y);
    printf("register Y increased to 0x%02x\n", cpu->reg_y);
    cpu->program_counter += 1;
    return 2;
}

/* C9 Compare memory with accumulator (immediate)
 *
 * A - M
 *
 * Compares the contents of the accumulator with an immediate 
 * value, and sets the zero, carry and negative flags as appropriate.
 * 
 * CARRY    : Set if A >= M (unsigned comparison)
 * ZERO     : Set if A == M
 * NEGATIVE : Set if the result is negative 
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int cmp_imm(cpu_6502 *cpu, operand_t *operand) {
    unsigned char result = cpu->accumulator - operand->byte[0];
    printf("comparing 0x%02x to 0x%02x\n", cpu->accumulator, operand->byte[0]);
    set_status_flags(cpu, result);
    if (cpu->accumulator >= operand->byte[0]) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }
    cpu->program_counter += 2;
    return 2;
}

/* CA Decrement index X by one (implied)
 *
 * X - 1 -> X
 * 
 * Decrements the value held by register X by one
 * 
 * Bytes:  1
 * Cycles: 2 
 */
unsigned int dex_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_x -= 1;
    printf("register X reduced to 0x%02x\n", cpu->reg_x);
    set_status_flags(cpu, cpu->reg_x);   
    cpu->program_counter += 1;
    return 2;
}

/* D0 Branch on result not zero (relative)
 *
 * Branch on Z = 0
 *
 * If the zero flag is clear, then change
 * the value of the program counter by the 
 * value of the operand
 * 
 * Bytes:  2
 * Cycles: 2 if no branching operation occurs
 *         3 if the branching operation occurs
 *         4 if the branching operation occurs 
 *           and the destination is on a new page
 * 
 * note: A page boundry crossing occurs when the
 *       branch destination is on a different page 
 *       than the instruction AFTER the branch 
 *       instruction
 */

unsigned int bne_rel(cpu_6502 *cpu, operand_t *operand) {
    unsigned short jump_vector = cpu->program_counter + 2;
    unsigned int cycles = 2;
    if (!(FLAG(cpu, ZERO))) {
        // must cast to char since jumps can be negative
        jump_vector += (char)operand->byte[0];
        // add an extra cycle since the branch succeeded
        // and add an additional cycle if the program 
        // counter is moved to a new page
        cycles += (1 + page_changed(cpu->program_counter + 2, jump_vector)); 
        printf("branching to 0x%04x\n", jump_vector);
    } else {
        printf("zero flag was not clear, no action taken\n");
    }
    cpu->program_counter = jump_vector;
    return cycles;
}

/* D8 Clear decimal mode
 *
 * 0 -> D
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int cld_impl(cpu_6502 *cpu, operand_t *operand) {
    CLEAR_FLAG(cpu, DECIMAL);
    cpu->program_counter += 1;
    printf("decimal status bit cleared\n");
    return 2;
}

/* E0 Compare memory with index X (immediate)
 *
 * X - M
 *
 * Compares the contents of the X register with an immediate 
 * value, and sets the zero, carry and negative flags as appropriate.
 * 
 * CARRY    : Set if X >= M (unsigned comparison)
 * ZERO     : Set if X == M
 * NEGATIVE : Set if the result is negative 
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int cpx_imm(cpu_6502 *cpu, operand_t *operand) {
    unsigned char result = cpu->reg_x - operand->byte[0];
    printf("comparing 0x%02x to 0x%02x\n", cpu->reg_x, operand->byte[0]);
    set_status_flags(cpu, result);
    if (cpu->reg_x >= operand->byte[0]) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }
    cpu->program_counter += 2;
    return 2;
}

/* EE Increment memory by one
 *
 * M + 1 -> M
 * 
 * Bytes:  1
 * Cycles: 5
 */

unsigned int inc_abs(cpu_6502 *cpu, operand_t *operand) {
    unsigned char value = mm_read(cpu->memory_map, operand->address) + 1;
    printf("increasing value at location 0x%04x to 0x%02x ", value, operand->address);
    mm_write(cpu->memory_map, operand->address, value);
    set_status_flags(cpu, value);
    cpu->program_counter += 3;
    return 5;
}

operation_t instruction_set[0x100] = {
    /* 00 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, ora_imm, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 10 */    bpl_rel, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 20 */    jsr_abs, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, and_imm, NULL, NULL, bits_abs, NULL, NULL, NULL,
    /* 30 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 40 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, jmp_abs, NULL, NULL, NULL,
    /* 50 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 60 */    rts_impl, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 70 */    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, sei_impl, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* 80 */    NULL, NULL, NULL, NULL, NULL, sta_zpg, stx_zpg, NULL, dey_impl, NULL, txa_impl, NULL, NULL, sta_abs, NULL, NULL,
    /* 90 */    NULL, sta_indy, NULL, NULL, NULL, NULL, NULL, NULL, NULL, sta_absy, txs_impl, NULL, NULL, NULL, NULL, NULL,
    /* A0 */    ldy_imm, NULL, ldx_imm, NULL, NULL, NULL, NULL, NULL, NULL, lda_imm, NULL, NULL, NULL, lda_abs, NULL, NULL,
    /* B0 */    bcs_rel, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, lda_absx, NULL, NULL,
    /* C0 */    cpy_imm, NULL, NULL, NULL, NULL, NULL, NULL, NULL, iny_impl, cmp_imm, dex_impl, NULL, NULL, NULL, NULL, NULL,
    /* D0 */    bne_rel, NULL, NULL, NULL, NULL, NULL, NULL, NULL, cld_impl, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    /* E0 */    cpx_imm, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, inc_abs, NULL,
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

int _6502_execute(cpu_6502 *cpu) {   
    printf("0x%04x\n", cpu->program_counter);
    unsigned char opcode, operand[2];
    opcode     = mm_read(cpu->memory_map, cpu->program_counter);
    operand[0] = mm_read(cpu->memory_map, cpu->program_counter + 1);
    operand[1] = mm_read(cpu->memory_map, cpu->program_counter + 2);
    printf("0x%02x ----------------------------------------\n", opcode);
    instruction_set[opcode](cpu, (operand_t*)&operand);
    return cpu->program_counter != 0x8057;
}

void _6502_reset(cpu_6502 *cpu) {
    // lo byte of reset vector located at: 0xfffc
    // hi byte of reset vector located at: 0xfffd
    unsigned short reset_vector = (mm_read(cpu->memory_map, 0xfffd) << 8) + mm_read(cpu->memory_map, 0xfffc);
    cpu->program_counter = reset_vector;
}
