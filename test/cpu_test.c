#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <time.h>

#include "../src/memory_map.h"
#include "../src/6502.c"

//#define TEST_DEBUG

typedef unsigned int (*test_t)(cpu_6502*);

void print_cpu_status(cpu_6502 *cpu) {
#ifdef TEST_DEBUG
    printf("############### CPU STATUS ###############\n");
    printf("PC: 0x%04x\n", cpu->program_counter);
    printf("SP: 0x%02x\n", cpu->stack_pointer);
    printf("A: 0x%02x X: 0x%02x Y: 0x%02x\n", cpu->accumulator, cpu->reg_x, cpu->reg_y);
    printf("N: %i V: %i B: %i D: %i I: %i Z: %i C: %i\n",
            FLAG(cpu, NEGATIVE),
            FLAG(cpu, OVERFLOW), 
            FLAG(cpu, BREAK), 
            FLAG(cpu, DECIMAL), 
            FLAG(cpu, INTERRUPT), 
            FLAG(cpu, ZERO), 
            FLAG(cpu, CARRY)  
          );
    printf("##########################################\n");
#endif
}

void reset_cpu(cpu_6502 *cpu, unsigned char *memory) {
    // reset registers
    cpu->program_counter = 0x0;
    cpu->accumulator     = 0x0;
    cpu->reg_y           = 0x0;
    cpu->reg_x           = 0x0;
    cpu->status_register = 0x0;
    cpu->stack_pointer   = 0x0;

    memset(memory, 0x0, 0x10000);

    sei_impl(cpu, NULL);
    cld_impl(cpu, NULL);
}

void run_test(test_t test, cpu_6502 *cpu, unsigned char *memory) {
    reset_cpu(cpu, memory);
    assert(test(cpu));   
}

/* Tests that ore_imm successfully sets the value of 
 * the accumulator equal to the result of itself ORed 
 * with the operand
 * 
 * opcode: 05
 */

unsigned int ora_zpg_test(cpu_6502 *cpu) {
    for (unsigned char addr = 0x1;; addr += 0x1) {
        
        // generate a value for the accumulator
        unsigned char A = rand() % 0x100;
        cpu->accumulator = A;
        
        // generate a value and write it to the zeropage
        // memory location intended to be read from
        unsigned char written_value = rand() % 0x100;
        mm_write(cpu->memory_map, addr, written_value);

        // execute instruction
        unsigned char operand[2] = {addr, 0x0};
        unsigned int cycles      = ora_zpg(cpu, (operand_t*)&operand);

        // verify the accumulator holds the correct value
        if (cpu->accumulator != (A | written_value)) return 0;

        // verify the zero flag is correctly set
        if (cpu->accumulator && FLAG(cpu, ZERO)) return 0;
        
        // verify the negative flag is correctly set
        if ((cpu->accumulator & 0x80) && !FLAG(cpu, NEGATIVE)) return 0;

        print_cpu_status(cpu);
        if (!addr) break;
    }
    printf("test 'ora_zpg_test' passed\n");
    return 1;    
}

/* Tests that ore_imm successfully sets the value of 
 * the accumulator equal to the result of itself ORed 
 * with value stored at the zeropage memory location 
 * indicated by the operand
 * 
 * opcode: 09
 */

unsigned int ora_imm_test(cpu_6502 *cpu) {
    for (unsigned char A = 0x1;; A += 0x1) {
        for (unsigned char oper = 0x1;; oper += 0x1) {
            
            // execute instruction
            cpu->accumulator = A;
            unsigned char operand[2] = {oper, 0x0};
            unsigned int  cycles     = ora_imm(cpu, (operand_t*)&operand);

            // verify the accumulator holds the correct value
            if (cpu->accumulator != (A | oper)) return 0;

            // verify the zero flag is correctly set
            if (cpu->accumulator && FLAG(cpu, ZERO)) return 0;
        
            // verify the negative flag is correctly set
            if ((cpu->accumulator & 0x80) && !FLAG(cpu, NEGATIVE)) return 0;

            print_cpu_status(cpu);
            if (!oper) break;
        }
        if (!A) break;
    }
    printf("test 'ora_imm_test' passed\n");
    return 1;
}

/* Tests that asl_acc correctly shifts the value held by the accumulator
 * one bit to the left. This test also verifies that the Z, N, and C status
 * flags are correctly set after the operation is performed, where:
 *
 * Z = (result == 0x0)
 * N = (result & 0x80)
 * C = 7th bit of the accumulator before the shift occurred
 * 
 * opcode: 0A
 */

unsigned int asl_acc_test(cpu_6502 *cpu) {
    for (unsigned char A = 0x1;; A += 0x1) {
        // set accumulator
        cpu->accumulator = A;
        unsigned char operand[2] = {0x0, 0x0};
        
        // repeat executing the instruction until all 8 bits have
        // been shifted
        unsigned char expected_value = A;
        for (int i = 0; i < 8; i += 1) {
            unsigned int cycles = asl_acc(cpu, (operand_t*)&operand);
            unsigned int carry_test = expected_value << 1;
            expected_value <<= 1;

            // verify the accumulator holds the correct value
            if (cpu->accumulator != expected_value) return 0;

            // verify the zero flag is correctly set
            if (cpu->accumulator && FLAG(cpu, ZERO)) return 0;
        
            // verify the negative flag is correctly set
            if ((cpu->accumulator & 0x80) && !FLAG(cpu, NEGATIVE)) return 0;

            // verify the carry flag is correctly set. The carry flag
            // is set if the value after the shift is too large to be
            // stored inside 8 bits
            if (carry_test > 0xff) {
                if (!FLAG(cpu, CARRY)) return 0;
            } else {
                if (FLAG(cpu, CARRY)) return 0;
            }

            print_cpu_status(cpu);
        }  
        if (!A) break;
    }
    printf("test 'asl_acc_test' passed\n");
    return 1;     
}

/* Tests that bpl_rel correctly modifies the value of the
 * program counter whether the condition for the branch 
 * is met or not. Also verifies that the correct number of
 * cycles are returned by the function.
 * 
 * If the condition is not met:
 *      cycles: 2
 *      pc = pc + 2 
 *
 * If the condition is met:
 *      cycles: 3 if no page boundry is crossed
 *      cycles: 4 if a page boundry is crossed
 *      pc = pc + 2 + operand
 * 
 * opcode: 10
 */

unsigned int bpl_rel_test(cpu_6502 *cpu) {
    // check every possibly offset, from every possible address
    // when the condition is both met and not met
    for (unsigned short addr = 0x1;; addr += 0x1) {
        for (unsigned char offset = 0x1;; offset += 0x1) {
            for (int cond = 0x0; cond < 0x2; cond += 0x1) {
                
                // switch the condition
                if (cond) {
                    CLEAR_FLAG(cpu, NEGATIVE);
                } else {
                    SET_FLAG(cpu, NEGATIVE);
                }
                
                // execute the instruction
                cpu->program_counter = addr;
                unsigned char operand[2] = {offset, 0x0};
                unsigned int cycles = bpl_rel(cpu, (operand_t*)&operand);
                
                // verify that the correct address is branched to
                unsigned short expected_address = addr + 2 + (cond * (char)offset);
                if (cpu->program_counter != expected_address) return 0; 
            }
            if (!offset) break;
        }
        print_cpu_status(cpu);
        if (!addr) break;
    }
    printf("test 'bpl_rel_test' passed\n");
    return 1;
}

/* Tests that clc_impl successfully clears the carry flag
 *
 * opcode: 18
 */

unsigned int clc_impl_test(cpu_6502 *cpu) {
    unsigned char operand[2] = {0x0, 0x0};
    unsigned int cycles;

    // verify the flag switches from off to off
    CLEAR_FLAG(cpu, CARRY);
    cycles = clc_impl(cpu, (operand_t*)&operand);
    if (FLAG(cpu, CARRY)) return 0;

    // verify the flag switch from on to off
    SET_FLAG(cpu, CARRY);
    cycles = clc_impl(cpu, (operand_t*)&operand);
    if (FLAG(cpu, CARRY)) return 0;

    printf("test 'clc_impl_test' passed\n");

    return 1;
}

/* Tests that sec_impl successfully sets the carry flag
 *
 * opcode: 38
 */

unsigned int sec_impl_test(cpu_6502 *cpu) {
    unsigned char operand[2] = {0x0, 0x0};
    unsigned int cycles;

    // verify the flag switches from off to on
    CLEAR_FLAG(cpu, CARRY);
    cycles = sec_impl(cpu, (operand_t*)&operand);
    if (!FLAG(cpu, CARRY)) return 0;

    // verify the flag switch from on to on
    SET_FLAG(cpu, CARRY);
    cycles = sec_impl(cpu, (operand_t*)&operand);
    if (!FLAG(cpu, CARRY)) return 0;

    printf("test 'sec_impl_test' passed\n");

    return 1;
}

/* Tests that sei_impl successfully sets the interrupt flag
 *
 * opcode: 78
 */

unsigned int sei_impl_test(cpu_6502 *cpu) {
    unsigned char operand[2] = {0x0, 0x0};
    unsigned int cycles;

    // verify the flag switches from off to on
    CLEAR_FLAG(cpu, INTERRUPT);
    cycles = sei_impl(cpu, (operand_t*)&operand);
    if (!FLAG(cpu, INTERRUPT)) return 0;

    // verify the flag switch from on to on
    SET_FLAG(cpu, INTERRUPT);
    cycles = sei_impl(cpu, (operand_t*)&operand);
    if (!FLAG(cpu, INTERRUPT)) return 0;

    printf("test 'sei_impl_test' passed\n");

    return 1;
}

/* Tests that sta_zpg successfully writes the value of the 
 * accumulator to all possible zeropage address locations
 *
 * opcode: 85
 */

unsigned int sta_zpg_test(cpu_6502 *cpu) {
    for (unsigned char addr = 0x1;; addr += 0x1) {
        
        // execute instruction
        cpu->accumulator = rand() % 0x100;
        unsigned char operand[2] = {addr, 0x0};
        unsigned int  cycles     = sta_zpg(cpu, (operand_t*)&operand);
        
        // verification
        unsigned read_value = mm_read(cpu->memory_map, addr);

        // verify the accumulator is equal to the written value
        if (cpu->accumulator != read_value) {
            printf("ADDRESS: 0x%04x WRITTEN_VALUE: 0x%02x ACTUAL_VALUE: 0x%02x\n", addr, cpu->accumulator, read_value);
            return 0;
        }

        print_cpu_status(cpu);
        if (!addr) break;
    }
    printf("test 'sta_zpg_test' passed\n");
    return 1;    
}

/* Tests that lda_zpg successfully sets the value held  
 * by the accumulator equal to the value stored at the
 * zeropage memory location indicated by the operand
 * 
 * opcode: A5
 */

unsigned int lda_zpg_test(cpu_6502 *cpu) {
    for (unsigned char addr = 0x1;; addr += 0x1) {
        
        // generate a value and write it to the zeropage
        // memory location intended to be read from
        unsigned char written_value = rand() % 0x100;
        mm_write(cpu->memory_map, addr, written_value);

        // execute instruction
        unsigned char operand[2] = {addr, 0x0};
        unsigned int cycles      = lda_zpg(cpu, (operand_t*)&operand);

        // verify the accumulator is equal to the written value
        if (cpu->accumulator != written_value) return 0;

        // verify the zero flag is correctly set
        if (cpu->accumulator && FLAG(cpu, ZERO)) return 0;
        
        // verify the negative flag is correctly set
        if ((cpu->accumulator & 0x80) && !FLAG(cpu, NEGATIVE)) return 0;

        if (!addr) break;
    }
    printf("test 'lda_zpg_test' passed\n");
    return 1;       
}

/* Tests that lda_imm successfully sets the value held  
 * by the accumulator equal to the operand
 *
 * opcode: A9
 */

unsigned int lda_imm_test(cpu_6502 *cpu) {
    for (unsigned char input = 0x1;; input += 0x1) {
        // execute instruction
        unsigned char operand[2] = {input, 0x0};
        unsigned int  cycles     = lda_imm(cpu, (operand_t*)&operand);
        
        // verify the accumulator is always equal to the input
        if (cpu->accumulator != input) return 0;
        
        // verify the zero flag is correctly set
        if (cpu->accumulator && FLAG(cpu, ZERO)) return 0;
        
        // verify the negative flag is correctly set
        if ((cpu->accumulator & 0x80) && !FLAG(cpu, NEGATIVE)) return 0;
        
        print_cpu_status(cpu);
        if (!input) break;
    }
    printf("test 'lda_imm_test' passed\n");
    return 1;
}

/* Tests that cld_impl successfully clears the decimal flag
 *
 * opcode: D8
 */

unsigned int cld_impl_test(cpu_6502 *cpu) {
    unsigned char operand[2] = {0x0, 0x0};
    unsigned int cycles;

    // verify the flag switches from off to off
    CLEAR_FLAG(cpu, DECIMAL);
    cycles = cld_impl(cpu, (operand_t*)&operand);
    if (FLAG(cpu, DECIMAL)) return 0;

    // verify the flag switch from on to off
    SET_FLAG(cpu, DECIMAL);
    cycles = cld_impl(cpu, (operand_t*)&operand);
    if (FLAG(cpu, DECIMAL)) return 0;

    printf("test 'cld_impl_test' passed\n");

    return 1;
}

int main() {

    srand(time(NULL));

    // create necessary hardware components in order to run 
    // tests. This includes a a block of memory in order to
    // test memory read / write operations, and the a 6502
    // to execute the instructions which are being tested
    memory_map_t  *mm = create_memory_map(0x10000);
    unsigned char *memory = calloc(1, 0x10000);
    mm_add_node(mm, &memory, 0x10000);
    cpu_6502 *cpu = create_6502(mm);

    assert(sei_impl_test(cpu));
    assert(cld_impl_test(cpu));

    run_test(lda_imm_test,  cpu, memory);
    run_test(ora_imm_test,  cpu, memory);
    run_test(sta_zpg_test,  cpu, memory);
    run_test(lda_zpg_test,  cpu, memory);
    run_test(ora_zpg_test,  cpu, memory);
    run_test(asl_acc_test,  cpu, memory);
    run_test(clc_impl_test, cpu, memory);
    run_test(sec_impl_test, cpu, memory);
    run_test(bpl_rel_test, cpu, memory);

    return 0;
}