#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "6502.h"

// #define CPU_DEBUG

#define SET_FLAG(C, X) C->status_register |= (0x1 << X)
#define CLEAR_FLAG(C, X) C->status_register &= (0xff - (0x1 << X))
#define FLAG(C, X) (C->status_register & (0x1 << X))

enum status_flag_t {
    CARRY,
    ZERO,
    INTERRUPT,
    DECIMAL,
    BREAK,
    OVERFLOW = 6,
    NEGATIVE
};

// prototype definitions of certain instructions which 
// are used in other instructions to reduce code size
unsigned int adc_imm(cpu_6502 *cpu, operand_t *operand);
unsigned int ora_imm(cpu_6502 *cpu, operand_t *operand);
unsigned int eor_imm(cpu_6502 *cpu, operand_t *operand);

void set_status_flags(cpu_6502 *cpu, unsigned char reg) {
    if (reg & 0x80) {
        SET_FLAG(cpu, NEGATIVE);
#ifdef CPU_DEBUG
        printf("negative status bit set\n");
#endif
    } else {
        CLEAR_FLAG(cpu ,NEGATIVE);
#ifdef CPU_DEBUG
        printf("negative status bit cleared\n");
#endif
    }
    
    if (!reg) {
        SET_FLAG(cpu, ZERO);
#ifdef CPU_DEBUG
        printf("zero status bit set\n");
#endif
    } else {
        CLEAR_FLAG(cpu, ZERO);
#ifdef CPU_DEBUG
        printf("zero status bit cleared\n");
#endif
    }
}

void push(cpu_6502 *cpu, unsigned char byte) {
    unsigned short stack_location = cpu->stack_pointer + 0x100;
    printf("pushing 0x%02x to stack location 0x%04x\n", byte, stack_location);
    mm_write(cpu->memory_map, stack_location, byte);
    cpu->stack_pointer -= 1;
}

unsigned char pop(cpu_6502 *cpu) {
    cpu->stack_pointer += 1;
    unsigned short stack_location = cpu->stack_pointer + 0x100;
    unsigned char byte = mm_read(cpu->memory_map, stack_location);
    printf("popping element 0x%02x from stack at location 0x%04x\n", byte, stack_location);
    return byte;
}

unsigned short pop_address(cpu_6502 *cpu) {
    return pop(cpu) + (pop(cpu) * 0x100);
}

/* determines if a page boundry has been crossed
 *
 * return: 1 if a page change has occured, otherwise 0
 */
unsigned int page_crossed(unsigned short current_address, unsigned short target_address) {
    unsigned int current_page = current_address / 0x100;
    unsigned int target_page  = target_address  / 0x100;
    return current_page != target_page;
}

/* fetches an address from a given location in memory
 *
 * return: The address stored at the given location in memory
 *         where the lo-byte of the address is stored at the
 *         given location and the hi-byte of the address is 
 *         stored at the given location + 1
 * 
 *         e.g:
 *      
 *         | given_location | given_location + 1|
 *         |  ADDR-LO-BYTE  |  ADDR-HI-BYTE     |
 * 
 *         fetched_address = (ADDR-HI-BYTE * 0x100) + ADDR-LO-BYTE
 */

static unsigned short fetch_address(cpu_6502 *cpu, unsigned short address_location) {
    return ((mm_read(cpu->memory_map, address_location + 1) << 8) + mm_read(cpu->memory_map, address_location));
}

/* executes the 'bit' instruction.
 *
 * e.g:
 *
 * BIT operand
 * 
 * This instruction sets the negative and overflow flags equal to bits 7 
 * and 6 of the operand respectively. It will also set the zero status flag 
 * active if the result of ANDing the operand with the value held by the 
 * accumulator is zero.
 * 
 * e.g:
 * 
 * Z = !(A & operand)
 * 
 */

void bit_exec(cpu_6502 *cpu, unsigned char value) {
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
}

unsigned char ror_exec(cpu_6502 *cpu, unsigned char value) {
    // store the 0th bit of the value so it can be used
    // later to determine the state of the carry flag 
    // after the operation
    unsigned char old_bit_0 = value & 0x01; 
    // store the current state of the carry flag so it
    // can be used later to determine what the 7th bit
    // of the value will be
    unsigned char new_bit_7 = FLAG(cpu, CARRY) ? 1 : 0;
    // shift the value right one bit, set the 7th 
    // bit to the value of the carry and update
    // memory with the modified value
    value >>= 1;
    value |= (new_bit_7 * 0x80);
    
    // determine the state of the zero and negative flags
    set_status_flags(cpu, value);

    // determine the state of the carry flag
    if (old_bit_0) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    return value;
}

unsigned char rol_exec(cpu_6502 *cpu, unsigned char value) {
    // store the 7th bit of the value so it can be used
    // later to determine the state of the carry flag 
    // after the operation
    unsigned char old_bit_7 = value & 0x80; 
    // store the current state of the carry flag so it
    // can be used later to determine what the 0th bit
    // of the value will be
    unsigned char new_bit_0 = FLAG(cpu, CARRY) ? 1 : 0;
    // shift the value left one bit and set the 0th 
    // bit to the value of the carry
    value <<= 1;
    value |= new_bit_0;
    
    // determine the state of the zero and negative flags
    set_status_flags(cpu, value);

    // determine the state of the carry flag
    if (old_bit_7) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    return value;
}

/* performs a logical bit shift one place to the right
 * on the given value. The resulting value is used to 
 * determine the state of the zero, negative and carry
 * flags where:
 * 
 * Z = value == 0x0 
 * N = value & 0x80
 * C = 0th bit of value before shift  
 */

unsigned int lsr_exec(cpu_6502 *cpu, unsigned char value) {
    // store the 0th bit of the value before the shift
    // so it can be used later to determine the state
    // of the carry flag
    unsigned char old_bit_0 = value & 0x01;
    
    // perform the shift
    value >>= 1;

    // determine the state of the zero and negative flags
    set_status_flags(cpu, value);
    
    // set the carry flag equal to the old 0th bit
    // of the value
    if (old_bit_0) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    return value;
}

/* performs a logical bit shift one place to the left
 * on the given value. The resulting value is used to 
 * determine the state of the zero, negative and carry
 * flags where:
 * 
 * Z = value == 0x0 
 * N = value & 0x80
 * C = 7th bit of value before shift  
 */

unsigned int asl_exec(cpu_6502 *cpu, unsigned char value) {
    // store the 7th bit of the value before the shift
    // so it can be used later to determine the state
    // of the carry flag
    unsigned char old_bit_7 = value & 0x80;
    
    // perform the shift
    value <<= 1;

    // determine the state of the zero and negative flags
    set_status_flags(cpu, value);
    
    // set the carry flag equal to the old 7th bit
    // of the value
    if (old_bit_7) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    return value;
}

/* Performs a branching operation which sets the program counter equal
 * to the position of the instruction following the branch instruction
 * plus the given offset (the offset can be negative). 
 * 
 * e.g: PC = PC + 2 + offset
 * 
 * The program will branch if the given condition is met, and will move
 * to the next instruction as normal if the condition is not met.
 * 
 * A branch instruction always takes at least 2 cycles, however they can 
 * take 3 or 4 cycles depdning on certain conditions.
 * 
 *      - If the condition IS NOT met, then no branch takes place, and 
 *        the instruction takes 2 cycles.
 * 
 *      - If the condition IS met, then the branch does take place, and 
 *        the instruction takes at least 3 cycles.
 * 
 *      - If the condition IS met and the branch destination is on a 
 *        different page than the instruction AFTER the branch instruction,
 *        then the instruction takes 4 cycles
 * 
 *      cond not met: 2 cycles
 *      cond met, destination same page: 3 cycles
 *      cond met, destination different page: 4 cycles
 */

unsigned int branch(cpu_6502 *cpu, char offset, unsigned int condition) {
    // The branch is made relative to the instruction following
    // the branch instruction, not the branch instruction itself
    unsigned short base_address = cpu->program_counter + 2;
    unsigned short jump_vector  = base_address;
    // Branch operation are always at least 2 cycles
    unsigned int cycles = 2;
    if (condition) {
        // apply the offset to the jump vector
        jump_vector += offset;
        // add an extra cycle since the branch succeeded
        // and add an additional cycle if the program 
        // counter is moved to a new page
        cycles += (1 + page_crossed(base_address, jump_vector));
#ifdef CPU_DEBUG
        printf("branching to 0x%04x\n", jump_vector);
#endif
    } else {
#ifdef CPU_DEBUG
        printf("condition not met, branch not taken\n");
#endif
    }
    cpu->program_counter = jump_vector;
    return cycles;
}

/* 05 OR memory with accumulator (zeropage)
 * 
 * A OR M -> A
 * 
 * Sets the value of the accumulator equal to result of
 * the value currently held by the accumulator ORed with
 * the value which is fetched from the zeropage memory
 * location given by the operand
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int ora_zpg(cpu_6502 *cpu, operand_t *operand) {
    // fetch the operand of the OR operation from the given
    // zeropage memory location and perform the operation 
    // using the immediate mode instruction
    unsigned char new_operand[2]= {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, operand->byte[0]);
    // make sure to add one additional cycle to account for the fact
    // that this is a zeropage operation, not an immediate one
    return ora_imm(cpu, (operand_t*)&new_operand) + 1;
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
#ifdef CPU_DEBUG
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
#endif
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 2;
}

/* 0A Shift left one bit (accumulator)
 *
 * A << 1 -> A
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int asl_acc(cpu_6502 *cpu, operand_t *operand) {
    // store bit 7 of the accumulator before the shift
    unsigned char old_bit_7 = cpu->accumulator & 0x80;
    cpu->accumulator <<= 1;

#ifdef CPU_DEBUG
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
#endif

    // determine the state of the zero and negative 
    // status flags
    set_status_flags(cpu, cpu->accumulator);
    
    // set the carry flag equal to the old 7th bit
    // of the accumulator
    if (old_bit_7) {
        SET_FLAG(cpu, CARRY);
#ifdef CPU_DEBUG
        printf("carry status bit set\n");
#endif
    } else {
        CLEAR_FLAG(cpu, CARRY);
#ifdef CPU_DEBUG
        printf("carry status bit cleared\n");
#endif
    }

    cpu->program_counter += 1;
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
    return branch(cpu, operand->byte[0], !FLAG(cpu, NEGATIVE));
}

/* 18 Clear carry flag
 *
 * 0 -> C
 *
 * Sets the carry flag to 0
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int clc_impl(cpu_6502 *cpu, operand_t *operand) {
    CLEAR_FLAG(cpu, CARRY);
#ifdef CPU_DEBUG
    printf("carry status bit cleared\n");
#endif
    cpu->program_counter += 1;  
    return 2;
}

/* 20 Jump to subroutine (absolute)
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

/* 24 Tests bits in memory with accumulator (zeropage)
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
 * Bytes:  2
 * Cycles: 3
 */

unsigned int bit_zpg(cpu_6502 *cpu, operand_t *operand) {
    // fetch value from memory
    unsigned char value = mm_read(cpu->memory_map, operand->byte[0]); 
    bit_exec(cpu, value);
    cpu->program_counter += 2;
    return 3;
}

/* 26 Rotate one bit left (zeropage)
 *
 * Bytes:  2
 * Cycles: 5
 */ 

unsigned int rol_zpg(cpu_6502 *cpu, operand_t *operand) {   
    // fetch the value from memory
    unsigned char target_address = operand->byte[0];
    unsigned char value = mm_read(cpu->memory_map, target_address);     
    // execute the rotate operation and store the modifed value in memory
    value = rol_exec(cpu, value);
    mm_write(cpu->memory_map, target_address, value);
    printf("ROL: memory location 0x%04x set to 0x%02x\n", target_address, value);
    cpu->program_counter += 2;
    return 5;
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

/* 2A Rotate one bit left (accumulator)
 *
 * Bytes:  1
 * Cycles: 2
 */ 

unsigned int rol_acc(cpu_6502 *cpu, operand_t *operand) {
    // store the 7th bit of the accumulator so it can
    // be used later to determine the state of the carry
    // flag after the operation
    unsigned char old_bit_7 = cpu->accumulator & 0x80;
    // store the current state of the carry flag so it
    // can be used later to determine what the 0th bit
    // of the accumulator will be
    unsigned char new_bit_0 = FLAG(cpu, CARRY) ? 1 : 0;

    // shift the value of the accumulator left one bit
    // and set the 0th bit to the value of the carry flag
    cpu->accumulator <<= 1;
    cpu->accumulator |= new_bit_0;

    // determine the state of the zero and negative flags
    set_status_flags(cpu, cpu->accumulator);

    // determine the state of the carry flag
    if (old_bit_7) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    cpu->program_counter += 1;
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

unsigned int bit_abs(cpu_6502 *cpu, operand_t *operand) {
    // fetch value from memory
    unsigned char value = mm_read(cpu->memory_map, operand->address); 
    bit_exec(cpu, value);
    cpu->program_counter += 3;
    return 4;
}

/* 30 Branch on minus plus (relative)
 *
 * Branch on N = 1
 *
 * If the negative flag is set, then change
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

unsigned int bmi_rel(cpu_6502 *cpu, operand_t *operand) {
    return branch(cpu, operand->byte[0], FLAG(cpu, NEGATIVE));
}

/* 38 Set carry flag
 *
 * 1 -> C
 *
 * Sets the carry flag to 1
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int sec_impl(cpu_6502 *cpu, operand_t *operand) {
    SET_FLAG(cpu, CARRY);
#ifdef CPU_DEBUG
    printf("carry status bit set\n");
#endif
    cpu->program_counter += 1;  
    return 2;
}

/* 3D AND memory with accumulator (absolute X)
 * 
 * A AND M -> A
 * 
 * Bytes:  3
 * Cycles: 4 if no page boundry is crossed
 *         5 if a page boundry is crossed
 */

unsigned int and_absx(cpu_6502 *cpu, operand_t *operand) {
    // fetch the operand of the AND operation from memory
    unsigned short base_address   = operand->address;
    unsigned short target_address = base_address + cpu->reg_x;
    unsigned char  new_operand[2] = {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, target_address);

    // perform the AND operation using the immediate
    // mode instruction, making sure to account for 
    // the additional cycles of an abosOlute x operation
    int cycles = and_imm(cpu, (operand_t*)&new_operand) + 2 + page_crossed(base_address, target_address);

    // increment the program counter an additional time to 
    // account for the extra byte in the instruction
    cpu->program_counter += 1;
    return cycles;
}

/* 40 Return from interrupt (implied)
 *
 * pull SR, pull PC
 * 
 * Returns the program to the location it was at
 * before the last interrupt occured
 * 
 * Bytes:  1
 * Cycles: 6
 */

unsigned int rti_impl(cpu_6502 *cpu, operand_t *operand) {
    // pop the return address off of the stack
    unsigned short jump_vector = pop_address(cpu);
    // restore the state of the status register before
    // the interrupt occurred
    cpu->status_register = pop(cpu);
    cpu->program_counter = jump_vector;
    return 7;
}

/* 45 XOR memory with accumulator (zeropage)
 * 
 * A XOR M -> A
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int eor_zpg(cpu_6502 *cpu, operand_t *operand) {
    // fetch the operand of the XOR operation from memory
    unsigned char  new_operand[2] = {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, operand->byte[0]);

    // perform the XOR operation using the immediate
    // mode instruction, making sure to account for 
    // the additional cycles of a zeropage operation
    return eor_imm(cpu, (operand_t*)&new_operand) + 1;
}

/* 46 Shift right one bit (zeropage)
 *
 * A >> 1 -> A
 * 
 * Bytes:  2
 * Cycles: 5
 */

unsigned int lsr_zpg(cpu_6502 *cpu, operand_t *operand) {
    // fetch the value from memory
    unsigned short target_address = operand->byte[0];
    unsigned char value = mm_read(cpu->memory_map, target_address);
    // execute the shift operation and store the modifed value in memory
    value = lsr_exec(cpu, value);
    mm_write(cpu->memory_map, target_address, value);
    printf("LSR: memory location 0x%04x set to 0x%02x\n", target_address, value);
    cpu->program_counter += 2;
    return 5;
}

/* 48 Push accumulator onto stack
 *
 * push A
 * 
 * Pushes the value of the accumulator onto the stack
 * 
 * Bytes:  1 
 * Cycles: 3
 */

unsigned int pha_impl(cpu_6502 *cpu, operand_t *operand) {
    push(cpu, cpu->accumulator);
    cpu->program_counter += 1;
    return 3;
}

/* 49 XOR memory with accumulator (immediate)
 * 
 * A XOR M -> A
 * 
 * Sets the value of the accumulator equal to the result
 * of the value currently held by the accumulator XORed 
 * with the given operand.
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int eor_imm(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator ^= operand->byte[0];
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 2;
}

/* 4A Shift right one bit (accumulator)
 *
 * A >> 1 -> A
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int lsr_acc(cpu_6502 *cpu, operand_t *operand) {
    // store the 0th bit of the accumulator before the 
    // shift so it can be used later to determine the 
    // state of the carry flag
    unsigned char old_bit_0 = cpu->accumulator & 0x01;
    
    // perform the shift
    cpu->accumulator >>= 1;
    printf("accumulator set to 0x%02x\n", cpu->accumulator);

    // determine the state of the zero and negative flags
    set_status_flags(cpu, cpu->accumulator);

    // set the carry flag equal to the old 0th bit
    // of the accumulator
    if (old_bit_0) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    cpu->program_counter += 1;
    return 2;
}

/* 4C Jump to new location (absolute)
 *
 * operand -> PC
 * 
 * Sets the program counter equal to the address given
 * by the operand
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

/* 50 Branch on overflow clear (relative)
 *
 * Branch on V = 0
 *
 * If the overflow flag is clear, then change
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

unsigned int bvc_rel(cpu_6502 *cpu, operand_t *operand) {
    return branch(cpu, operand->byte[0], !FLAG(cpu, OVERFLOW));
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
    unsigned short jump_vector = pop_address(cpu);
    printf("returning from subroutine to 0x%04x\n", jump_vector + 1);
    cpu->program_counter = jump_vector + 1;
    return 6;
}

/* 65 Adds memory to acccumulator with carry (zeropage)
 *
 * A + M + C -> A, C
 * 
 * Sets Negative flag if: Bit 7 of result is active
 * Sets Zero flag if    : result == 0x0
 * Sets Carry flag if   : result >  0xff
 * Sets Overflow flag if: ((A ^ result) & (M ^ result) & 0x80) is non zero
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int adc_zpg(cpu_6502 *cpu, operand_t *operand) {
    // fetch the second operand of the addition operation
    // from memory and perform the operation using the
    // immediate mode instruction
    unsigned char new_operand[2] = {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, operand->byte[0]);
    // make sure to add an additional cycle to account for the fact
    // that this is a zeropage operation, not an immediate one
    return adc_imm(cpu, (operand_t*)&new_operand) + 1;
}

/* 68 Pull accumulator from stack
 *
 * pull -> A
 * 
 * Pops the top element from the stack and assigns
 * it to the accumulator
 * 
 * Bytes:  1
 * Cycles: 4
 */

unsigned int pla_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = pop(cpu);
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 1;
    return 4;
}

/* 69 Adds memory to acccumulator with carry (immediate)
 *
 * A + M + C -> A, C
 * 
 * Sets Negative flag if: Bit 7 of result is active
 * Sets Zero flag if    : result == 0x0
 * Sets Carry flag if   : the operation overflows e.g: result > 0xff
 * Sets Overflow flag if: ((A ^ result) & (M ^ result) & 0x80) is non zero
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int adc_imm(cpu_6502 *cpu, operand_t *operand) {
    // store the actual result so it can be checked later
    // if it is necessary to set the carry flag. If the 
    // carry flag is set initially, increment the result
    // by one
    unsigned int result = cpu->accumulator + operand->byte[0];
    if (FLAG(cpu, CARRY)) {
        result += 1;
    }

    // store the old value of the accumulator so it can
    // be used later to check for signed overflow
    unsigned char old_a = cpu->accumulator;

    // store the (potentially truncated) result in the accumulator
    cpu->accumulator = (unsigned char)result;
    printf("accumulator set to 0x%02x\n", cpu->accumulator);

    // determine the state of the zero and negative flags
    set_status_flags(cpu, cpu->accumulator);

    // if the result is too large to be stored as an 8 bit 
    // integer, set the carry flag
    if (result > 0xff) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    // check for signed overflow. If the sign of both inputs is different
    // from the sign of the result, then an overflow has occured
    //
    // formula taken from:
    //  
    // http://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
    // 
    if ((old_a ^ cpu->accumulator) & (operand->byte[0] ^ cpu->accumulator) & 0x80) {
        SET_FLAG(cpu, OVERFLOW);
        printf("overflow status bit set\n");
    } else {
        SET_FLAG(cpu, OVERFLOW);
        printf("overflow status bit set\n");
    }

    // move the program counter to the to the next instruction
    // and return the number of cycles the instruction took
    cpu->program_counter += 2;
    return 2;
}

/* 6A Rotate one bit right (accumulator)
 *
 * Bytes:  1
 * Cycles: 2
 */ 

unsigned int ror_acc(cpu_6502 *cpu, operand_t *operand) {
    // store the 0th bit of the accumulator so it can
    // be used later to determine the state of the carry
    // flag after the operation
    unsigned char old_bit_0 = cpu->accumulator & 0x01;
    // store the current state of the carry flag so it
    // can be used later to determine what the 7th bit
    // of the accumulator will be
    unsigned char new_bit_7 = FLAG(cpu, CARRY) ? 1 : 0;

    // shift the value of the accumulator right one bit
    // and set the 7th bit to the value of the carry flag
    cpu->accumulator >>= 1;
    cpu->accumulator |= (new_bit_7 * 0x80);

    // determine the state of the zero and negative flags
    set_status_flags(cpu, cpu->accumulator);

    // determine the state of the carry flag
    if (old_bit_0) {
        SET_FLAG(cpu, CARRY);
        printf("carry status bit set\n");
    } else {
        CLEAR_FLAG(cpu, CARRY);
        printf("carry status bit cleared\n");
    }

    cpu->program_counter += 1;
    return 2;
}

/* 6C Jump to new location (indirect)
 *
 * (operand) -> PC
 * 
 * Sets the program counter equal to the address stored
 * at the address given by the operand
 * 
 * Bytes:  3
 * Cycles: 5
 */

unsigned int jmp_indr(cpu_6502 *cpu, operand_t *operand) {
    // fetch the target jump vector from memory
    unsigned short jump_vector = fetch_address(cpu, operand->address);
    printf("jumping to 0x%04x\n", jump_vector);
    cpu->program_counter = jump_vector;
    return 5;
}

/* 6D Adds memory to acccumulator with carry (absolute)
 *
 * A + M + C -> A, C
 * 
 * Sets Negative flag if: Bit 7 of result is active
 * Sets Zero flag if    : result == 0x0
 * Sets Carry flag if   : result >  0xff
 * Sets Overflow flag if: ((A ^ result) & (M ^ result) & 0x80) is non zero
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int adc_abs(cpu_6502 *cpu, operand_t *operand) {
    // fetch the second operand of the addition operation
    // from memory and perform the operation using the
    // immediate mode instruction
    unsigned char new_operand[2] = {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, operand->address);
    // make sure to add two additional cycles to account for the fact
    // that this is an absolute operation, not an immediate one
    int cycles = adc_imm(cpu, (operand_t*)&new_operand) + 2;
    // move the program counter forward an extra byte to account
    // for the additional instruction lengnth over the immediate
    // operation
    cpu->program_counter += 1;
    return cycles;
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
#ifdef CPU_DEBUG
    printf("interrupt status bit set\n");
#endif
    cpu->program_counter += 1;
    return 2;
}

/* 79 Adds memory to acccumulator with carry (absolute Y)
 *
 * A + M + C -> A, C
 * 
 * Sets Negative flag if: Bit 7 of result is active
 * Sets Zero flag if    : result == 0x0
 * Sets Carry flag if   : result >  0xff
 * Sets Overflow flag if: ((A ^ result) & (M ^ result) & 0x80) is non zero
 * 
 * Bytes:  3
 * Cycles: 4 if no page boundry is crossed
 *         5 if a page boundry is crossed
 */

unsigned int adc_absy(cpu_6502 *cpu, operand_t *operand) {
    // fetch the second operand of the addition operation from memory
    unsigned short base_address   = operand->address;
    unsigned short target_address = base_address + cpu->reg_y;
    unsigned char new_operand[2]  = {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, target_address);
    
    // perform the addition operation using the immediate
    // mode instruction, making sure to account for the 
    // additional cycles of an abosolute Y operation
    int cycles = adc_imm(cpu, (operand_t*)&new_operand) + 2 + page_crossed(base_address, target_address);
    
    // increment the program counter an additional time to 
    // account for the extra byte in the instruction
    cpu->program_counter += 1;
    return cycles;
}

/* 7E Rotate one bit right (absolute X)
 *
 * Bytes:  3
 * Cycles: 7
 */ 

unsigned int ror_absx(cpu_6502 *cpu, operand_t *operand) {   
    // fetch the value from memory
    unsigned short target_address = operand->address + cpu->reg_x;
    unsigned char value = mm_read(cpu->memory_map, target_address);     
    // execute the rotate operation and store the modifed value in memory
    value = ror_exec(cpu, value);
    mm_write(cpu->memory_map, target_address, value);
    printf("ROR: memory location 0x%04x set to 0x%02x\n", target_address, value);
    cpu->program_counter += 3;
    return 7;
}

/* 84 Store index Y in memory (zeropage)
 *
 * Y -> M
 * 
 * Stores the value held by register Y at the
 * memory location indicated by the operand
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int sty_zpg(cpu_6502 *cpu, operand_t *operand) {
    mm_write(cpu->memory_map, operand->byte[0], cpu->reg_y);
    printf("memory location 0x%04x set to 0x%02x\n", operand->byte[0], mm_read(cpu->memory_map, operand->byte[0]));
    cpu->program_counter += 2;
    return 3;
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
#ifdef CPU_DEBUG
    printf("memory location 0x%04x set to 0x%02x\n", operand->byte[0], mm_read(cpu->memory_map, operand->byte[0]));
#endif
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
 * Sets the value of the accumulator equal to the
 * current value of the X reegister
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int txa_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = cpu->reg_x;
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 1;
    return 2;
}

/* 8C Store index Y in memory (absolute)
 *
 * Y -> M
 * 
 * Stores the value held by register Y at the
 * memory location indicated by the operand
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int sty_abs(cpu_6502 *cpu, operand_t *operand) {
    mm_write(cpu->memory_map, operand->address, cpu->reg_y);
    printf("memory location 0x%04x set to 0x%02x\n", operand->address, mm_read(cpu->memory_map, operand->address));
    cpu->program_counter += 3;
    return 4;
}

/* 8D Store accumulator in memory (absolute)
 *
 * A -> M
 *
 * Stores the value held by the accumulator at the
 * memory location indicated by the operand
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

/* 8E Store index X in memory (absolute)
 *
 * X -> M
 * 
 * Stores the value held by the X register at the 
 * location in memory indicated by the operand.
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int stx_abs(cpu_6502 *cpu, operand_t *operand) {
    mm_write(cpu->memory_map, operand->address, cpu->reg_x);
    printf("memory location 0x%04x set to 0x%02x\n", operand->address, mm_read(cpu->memory_map, operand->address));
    cpu->program_counter += 3;
    return 4;
}

/* 90 Branch on carry clear (relative)
 *
 * Branch on C = 0
 *
 * If the carry flag is clear, then change
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

unsigned int bcc_rel(cpu_6502 *cpu, operand_t *operand) {
    return branch(cpu, operand->byte[0], !FLAG(cpu, CARRY));
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
    unsigned short target_address = fetch_address(cpu, operand->byte[0]); + cpu->reg_y;
    mm_write(cpu->memory_map, target_address, cpu->accumulator);
    printf("memory location 0x%04x set to 0x%02x\n", target_address, mm_read(cpu->memory_map, target_address));
    cpu->program_counter += 2;
    return 6;
}

/* 95 Store accumulator in memory (zeropage X)
 *
 * A -> M
 * 
 * Stores the value held by the accumlator at the 
 * location in memory indicated by the operand plus
 * the value held by the X register
 * 
 * Bytes:  2
 * Cycles: 4
 */

unsigned int sta_zpgx(cpu_6502 *cpu, operand_t *operand) {
    // store the target addres, making sure to store it in an
    // 8 bit location, since in the event of an overflow, the 
    // value is meant to wrap around in the case of zeropage
    // locations
    unsigned char target_address = operand->byte[0] + cpu->reg_x;
    mm_write(cpu->memory_map, target_address, cpu->accumulator);
    printf("memory location 0x%04x set to 0x%02x\n", target_address, mm_read(cpu->memory_map, target_address));
    cpu->program_counter += 2;
    return 4;
}

/* 98 transfer index Y to accumulator (implied)
 *
 * Y -> A
 * 
 * Sets the value of the accumulator equal to the
 * current value of the Y register
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int tya_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = cpu->reg_y;
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 1;
    return 2;
}

/* 99 Store accumulator in memory (absolute Y)
 *
 * A -> M
 * 
 * Stores the value held by the accumulator at the
 * location in memory given by the operand plus the
 * value held by register Y
 * 
 * e.g:
 * 
 * target_address = operand + Y
 * M[target_address] = A
 * 
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

/* 9D Store accumulator in memory (absolute X)
 *
 * A -> M
 * 
 * Stores the value held by the accumulator at the
 * location in memory given by the operand plus the
 * value held by register X
 * 
 * e.g:
 * 
 * target_address = operand + X
 * M[target_address] = A
 * 
 * Bytes:  3
 * Cycles: 5
 */

unsigned int sta_absx(cpu_6502 *cpu, operand_t *operand) {
    unsigned short target_address = operand->address + cpu->reg_x;
    mm_write(cpu->memory_map, target_address, cpu->accumulator);
    printf("memory location 0x%04x set to 0x%02x\n", target_address, mm_read(cpu->memory_map, target_address));
    cpu->program_counter += 3;
    return 5;
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

/* A4 Load index Y with memory (zeropage)
 *
 * M -> Y
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int ldy_zpg(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_y = mm_read(cpu->memory_map, operand->byte[0]);
    printf("register Y set to 0x%02x\n", cpu->reg_y);
    set_status_flags(cpu, cpu->reg_y);
    cpu->program_counter += 2;
    return 3;
}

/* A5 Load accumulator with memory (zeropage)
 *
 * M -> A
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int lda_zpg(cpu_6502 *cpu, operand_t *operand) {
    cpu->accumulator = mm_read(cpu->memory_map, operand->byte[0]);
#ifdef CPU_DEBUG
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
#endif
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 3;
}

/* A6 Load index X with memory (zeropage)
 *
 * M -> X
 * 
 * Fetches a value from the location in memory given
 * by the operand and stores the fetched value in the
 * X register
 * 
 * Bytes:  2
 * Cycles: 3
 */

unsigned int ldx_zpg(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_x = mm_read(cpu->memory_map, operand->byte[0]);
    printf("register X set to 0x%02x\n", cpu->reg_x);
    set_status_flags(cpu, cpu->reg_x);
    cpu->program_counter += 2;
    return 3;
}

/* A8 transfer accumulator to index Y (implied)
 *
 * A -> Y
 * 
 * Sets the value of register Y equal to the value
 * currently held by the accumulator
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int tay_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_y = cpu->accumulator;
    printf("register Y set to 0x%02x\n", cpu->reg_y);
    set_status_flags(cpu, cpu->reg_y);
    cpu->program_counter += 1;
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
#ifdef CPU_DEBUG
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
#endif
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 2;
}

/* AA transfer accumulator to index X (implied)
 *
 * A -> X
 * 
 * Sets the value of register X equal to the value
 * currently held by the accumulator
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int tax_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_x = cpu->accumulator;
    printf("register X set to 0x%02x\n", cpu->reg_x);
    set_status_flags(cpu, cpu->reg_x);
    cpu->program_counter += 1;
    return 2;
}

/* AC Load index Y with memory (absolute)
 *
 * M -> Y
 * 
 * Fetches a value from the location in memory given
 * by the operand and stores the fetched value in the
 * register Y
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int ldy_abs(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_y = mm_read(cpu->memory_map, operand->address);
    printf("register Y set to 0x%02x\n", cpu->reg_y);
    set_status_flags(cpu, cpu->reg_y);
    cpu->program_counter += 3;
    return 4;
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

/* AE Load index X with memory (absolute)
 *
 * M -> X
 * 
 * Fetches a value from the location in memory given
 * by the operand and stores the fetched value in the
 * X register
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int ldx_abs(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_x = mm_read(cpu->memory_map, operand->address);
    printf("register X set to 0x%02x\n", cpu->reg_x);
    set_status_flags(cpu, cpu->reg_x);
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
    return branch(cpu, operand->byte[0], FLAG(cpu, CARRY));
}

/* B1 Load accumulator with memory (indirect Y)
 *
 * M -> A
 * 
 * Featches a value from memory at a target address which is 
 * fetched from memory. The location of the lo-byte of the 
 * target address is given by the operand, and the location of
 * the hi-byte of the target addres is given by the operand + 1.
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
 * The value is then fetched from the final address and stored
 * in the accumulator
 * 
 * Bytes:  2
 * Cycles: 5 if no page boundry is crossed
 *         6 if a page boundry is crossed
 */

unsigned int lda_indy(cpu_6502 *cpu, operand_t *operand) {
    unsigned int cycles = 5;
    // fetch the base address from memory and
    // calculate the target address
    unsigned short base_address = fetch_address(cpu, operand->byte[0]);
    unsigned short target_address = base_address + cpu->reg_y;  
    cpu->accumulator = mm_read(cpu->memory_map, target_address);
    // add an additional cycle if the base address 
    // is on a different page to the address plus
    // the offset
    cycles += page_crossed(base_address, target_address);
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return cycles;
}

/* B5 Load accumulator with memory (zeropage X)
 *
 * M -> A
 * 
 * Bytes:  2
 * Cycles: 4
 */

unsigned int lda_zpgx(cpu_6502 *cpu, operand_t *operand) {
    // store the target addres, making sure to store it in an
    // 8 bit location, since in the event of an overflow, the 
    // value is meant to wrap around in the case of zeropage
    // locations
    unsigned char target_address = operand->byte[0] + cpu->reg_x;
    cpu->accumulator = mm_read(cpu->memory_map, target_address);
    printf("accumulator set to 0x%02x\n", cpu->accumulator);
    set_status_flags(cpu, cpu->accumulator);
    cpu->program_counter += 2;
    return 4;
}

/* B9 Load accumulator with memory (absolute Y)
 *
 * M -> A
 * 
 * Bytes:  3
 * Cycles: 4 if no page boundry is crossed
 *         5 if a page boundry is crossed
 */

unsigned int lda_absy(cpu_6502 *cpu, operand_t *operand) {
    unsigned int cycles = 4;
    unsigned short target_address = operand->address + cpu->reg_y;
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

/* BE Load index X with memory (absolute Y)
 *
 * M -> X
 * 
 * Bytes:  3
 * Cycles: 4 if no page boundry is crossed
 *         5 if a page boundry is crossed
 */

unsigned int ldx_absy(cpu_6502 *cpu, operand_t *operand) {
    unsigned int cycles = 4;
    unsigned short target_address = operand->address + cpu->reg_y;
    cpu->reg_x = mm_read(cpu->memory_map, target_address);
    // add an additional cycle if the base address
    // is on a different page to the address plus
    // the offset
    cycles += page_crossed(operand->address, target_address);
    printf("register X set to 0x%02x\n", cpu->reg_x);
    set_status_flags(cpu, cpu->reg_x);
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

/* C6 Decrement memory by one (zeropage)
 *
 * M - 1 -> M
 * 
 * Decreases the value held at the memory location
 * given by the operand by one
 * 
 * Bytes:  2
 * Cycles: 5
 */

unsigned int dec_zpg(cpu_6502 *cpu, operand_t *operand) {
    unsigned char value = mm_read(cpu->memory_map, operand->byte[0]) - 1;
    printf("decreasing value at location 0x%04x to 0x%02x\n", operand->byte[0], value);
    mm_write(cpu->memory_map, operand->byte[0], value);
    set_status_flags(cpu, value);
    cpu->program_counter += 2;
    return 5;
}

/* C8 Increment index Y by one (implied)
 *
 * Y + 1 -> Y
 * 
 * Increases the value held by register Y by one
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
 * Decreases the value held by register X by one
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

/* CD Compare memory with accumulator (absolute)
 *
 * A - M
 *
 * Compares the contents of the accumulator with a value fetched 
 * from the memory location given by the operand and sets the zero, 
 * carry and negative flags as appropriate.
 * 
 * CARRY    : Set if A >= M (unsigned comparison)
 * ZERO     : Set if A == M
 * NEGATIVE : Set if the result is negative 
 * 
 * Bytes:  3
 * Cycles: 4
 */

unsigned int cmp_abs(cpu_6502 *cpu, operand_t *operand) {
    // fetch the value from memory
    unsigned char new_operand[2] ={0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, operand->address);
    
    // perform the operation using the immediate instruction
    // making sure to add two additional cycles to account for
    // this being an absolute operation, not an immediate one
    int cycles = cmp_imm(cpu, (operand_t*)&new_operand) + 2;
    
    // increment the program counter an additional time to 
    // account for the extra byte in the instruction
    cpu->program_counter += 1;
    return cycles;
}

/* CE Decrement memory by one (absolute)
 *
 * M - 1 -> M
 * 
 * Decreases the value held at the memory location
 * given by the operand by one
 * 
 * Bytes:  3
 * Cycles: 6
 */

unsigned int dec_abs(cpu_6502 *cpu, operand_t *operand) {
    unsigned char value = mm_read(cpu->memory_map, operand->address) - 1;
    printf("decreasing value at location 0x%04x to 0x%02x\n", operand->address, value);
    mm_write(cpu->memory_map, operand->address, value);
    set_status_flags(cpu, value);
    cpu->program_counter += 3;
    return 6;
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
    return branch(cpu, operand->byte[0], !FLAG(cpu, ZERO));
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
#ifdef CPU_DEBUG
    printf("decimal status bit cleared\n");
#endif
    return 2;
}

/* D9 Compare memory with accumulator (absolute Y)
 *
 * A - M
 *
 * Compares the contents of the accumulator with a value fetched 
 * from the memory location given by the operand plus the value  
 * of the Y register and sets the zero, carry and negative flags 
 * as appropriate.
 * 
 * CARRY    : Set if A >= M (unsigned comparison)
 * ZERO     : Set if A == M
 * NEGATIVE : Set if the result is negative 
 * 
 * Bytes:  3
 * Cycles: 4 if no page boundry is crossed
 *         5 if a page boundry is crossed
 */

unsigned int cmp_absy(cpu_6502 *cpu, operand_t *operand) {
    // fetch the value from memory
    unsigned short base_address   = operand->address;
    unsigned short target_address = base_address + cpu->reg_y;
    unsigned char new_operand[2]  = {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, target_address);
    
    // perform the operation using the immediate instruction
    // making sure to add all additional cycles to account for
    // this being an absolute Y operation, not an immediate one
    int cycles = cmp_imm(cpu, (operand_t*)&new_operand) + 2 + page_crossed(base_address, target_address);
    
    // increment the program counter an additional time to 
    // account for the extra byte in the instruction
    cpu->program_counter += 1;
    return cycles;
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

/* E6 Increment memory by one (zeropage)
 *
 * M + 1 -> M
 * 
 * Bytes:  2
 * Cycles: 5
 */

unsigned int inc_zpg(cpu_6502 *cpu, operand_t *operand) {
    unsigned char value = mm_read(cpu->memory_map, operand->byte[0]) + 1;
    printf("increasing value at location 0x%04x to 0x%02x\n", operand->byte[0], value);
    mm_write(cpu->memory_map, operand->byte[0], value);
    set_status_flags(cpu, value);
    cpu->program_counter += 2;
    return 5;
}

/* E8 Increment index X by one
 *
 * X + 1 -> X
 * 
 * Increases the value held by register X by one
 * 
 * Bytes:  1
 * Cycles: 2
 */

unsigned int inx_impl(cpu_6502 *cpu, operand_t *operand) {
    cpu->reg_x += 1;
    set_status_flags(cpu, cpu->reg_x);
    printf("register X increased to 0x%02x\n", cpu->reg_x);
    cpu->program_counter += 1;
    return 2;
}

/* E9 Subtract memory from accumulator with borrow (immediate)
 *
 * A - M - C -> A
 * 
 * Sets Negative flag if: Bit 7 of result is active
 * Sets Zero flag if    : result == 0x0
 * Sets Carry flag if   : the operation does not underflow
 * Sets Overflow flag if: ((A ^ result) & ((0xff - M) ^ result) & 0x80) is non zero
 * 
 * Bytes:  2
 * Cycles: 2
 */

unsigned int sbc_imm(cpu_6502 *cpu, operand_t *operand) {
    // convert the second operand to ones complement
    // and doing an addition operation is the same as
    // performing a subtraction operation
    //
    // idea taken from:
    //  
    // http://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
    //
    unsigned char  new_operand[2] = {0x0, 0x0};
    new_operand[0] = 0xff - operand->byte[0];
    return adc_imm(cpu, (operand_t*)&new_operand);
}

/* EE Increment memory by one (absolute)
 *
 * M + 1 -> M
 * 
 * Bytes:  3
 * Cycles: 6
 */

unsigned int inc_abs(cpu_6502 *cpu, operand_t *operand) {
    unsigned char value = mm_read(cpu->memory_map, operand->address) + 1;
    printf("increasing value at location 0x%04x to 0x%02x\n", operand->address, value);
    mm_write(cpu->memory_map, operand->address, value);
    set_status_flags(cpu, value);
    cpu->program_counter += 3;
    return 6;
}

/* F0 Branch on result zero (relative)
 *
 * Branch on Z = 1
 *
 * If the zero flag is set, then change
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

unsigned int beq_rel(cpu_6502 *cpu, operand_t *operand) {
    return branch(cpu, operand->byte[0], FLAG(cpu, ZERO));
}

/* F9 Subtract memory from accumulator with borrow (absolute Y)
 *
 * A - M - C -> A
 * 
 * Sets Negative flag if: Bit 7 of result is active
 * Sets Zero flag if    : result == 0x0
 * Sets Carry flag if   : the operation does not underflow
 * Sets Overflow flag if: ((A ^ result) & ((0xff - M) ^ result) & 0x80) is non zero
 * 
 * Bytes:  3
 * Cycles: 4 if no page boundry is crossed
 *         5 if a page boundry is crossed
 */

unsigned int sbc_absy(cpu_6502 *cpu, operand_t *operand) {
    // fetch the second operand of the subtraction operation from memory
    unsigned short base_address   = operand->address;
    unsigned short target_address = base_address + cpu->reg_y;
    unsigned char  new_operand[2] = {0x0, 0x0};
    new_operand[0] = mm_read(cpu->memory_map, target_address);

    // perform the subtraction operation using the immediate
    // mode instruction, making sure to account for the additional
    // cycles of an abosolute Y operation
    int cycles = sbc_imm(cpu, (operand_t*)&new_operand) + 2 + page_crossed(base_address, target_address);

    // increment the program counter an additional time to 
    // account for the extra byte in the instruction
    cpu->program_counter += 1;
    return cycles;
}

operation_t instruction_set[0x100] = {
    //             00        01        02      03      04        05        06      07      08        09        0A      0B      0C        0D        0E      0F
    /* 00 */    NULL    , NULL    , NULL    , NULL, NULL    , ora_zpg , NULL    , NULL, NULL    , ora_imm , asl_acc , NULL, NULL    , NULL    , NULL    , NULL,
    /* 10 */    bpl_rel , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL, clc_impl, NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL,
    /* 20 */    jsr_abs , NULL    , NULL    , NULL, bit_zpg , NULL    , rol_zpg , NULL, NULL    , and_imm , rol_acc , NULL, bit_abs , NULL    , NULL    , NULL,
    /* 30 */    bmi_rel , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL, sec_impl, NULL    , NULL    , NULL, NULL    , and_absx, NULL    , NULL,
    /* 40 */    rti_impl, NULL    , NULL    , NULL, NULL    , eor_zpg , lsr_zpg , NULL, pha_impl, eor_imm , lsr_acc , NULL, jmp_abs , NULL    , NULL    , NULL,
    /* 50 */    bvc_rel , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL,
    /* 60 */    rts_impl, NULL    , NULL    , NULL, NULL    , adc_zpg , NULL    , NULL, pla_impl, adc_imm , ror_acc , NULL, jmp_indr, adc_abs , NULL    , NULL,
    /* 70 */    NULL    , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL, sei_impl, adc_absy, NULL    , NULL, NULL    , NULL    , ror_absx, NULL,
    /* 80 */    NULL    , NULL    , NULL    , NULL, sty_zpg , sta_zpg , stx_zpg , NULL, dey_impl, NULL    , txa_impl, NULL, sty_abs , sta_abs , stx_abs , NULL,
    /* 90 */    bcc_rel , sta_indy, NULL    , NULL, NULL    , sta_zpgx, NULL    , NULL, tya_impl, sta_absy, txs_impl, NULL, NULL    , sta_absx, NULL    , NULL,
    /* A0 */    ldy_imm , NULL    , ldx_imm , NULL, ldy_zpg , lda_zpg , ldx_zpg , NULL, tay_impl, lda_imm , tax_impl, NULL, ldy_abs , lda_abs , ldx_abs , NULL,
    /* B0 */    bcs_rel , lda_indy, NULL    , NULL, NULL    , lda_zpgx, NULL    , NULL, NULL    , lda_absy, NULL    , NULL, NULL    , lda_absx, ldx_absy, NULL,
    /* C0 */    cpy_imm , NULL    , NULL    , NULL, NULL    , NULL    , dec_zpg , NULL, iny_impl, cmp_imm , dex_impl, NULL, NULL    , cmp_abs , dec_abs , NULL,
    /* D0 */    bne_rel , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL, cld_impl, cmp_absy, NULL    , NULL, NULL    , NULL    , NULL    , NULL,
    /* E0 */    cpx_imm , NULL    , NULL    , NULL, NULL    , NULL    , inc_zpg , NULL, inx_impl, sbc_imm , NULL    , NULL, NULL    , NULL    , inc_abs , NULL,
    /* F0 */    beq_rel , NULL    , NULL    , NULL, NULL    , NULL    , NULL    , NULL, NULL    , sbc_absy, NULL    , NULL, NULL    , NULL    , NULL    , NULL,
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
    return instruction_set[opcode](cpu, (operand_t*)&operand);
}

void _6502_reset(cpu_6502 *cpu) {
    // lo byte of reset vector located at: 0xfffc
    // hi byte of reset vector located at: 0xfffd
    unsigned short reset_vector = fetch_address(cpu, 0xfffc);
    cpu->program_counter = reset_vector;
}

int _6502_nmi(cpu_6502 *cpu) {
    // lo byte of interrupt vector located at: 0xfffa
    // hi byte of interrupt vector located at: 0xfffb
    unsigned short interrupt_vector = fetch_address(cpu, 0xfffa);
    // push status register to stack
    push(cpu, cpu->status_register);
    // push the next instruction onto the stack
    push(cpu, (unsigned char)(cpu->program_counter >> 8)); // push hi byte
    push(cpu, (unsigned char)(cpu->program_counter)); // push lo byte
    cpu->program_counter = interrupt_vector;
    // return the cycle count for the interrupt
    return 7;
}
