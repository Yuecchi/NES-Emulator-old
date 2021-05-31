#ifndef _6502_H
#define _6502_H

#include "memory_map.h"

typedef struct _cpu_6502 {
    memory_map_t *memory_map;
    unsigned short program_counter;
    unsigned char accumulator;
    unsigned char reg_x;
    unsigned char reg_y;
    unsigned char status_register; // often referred to as 'P' in documentation
    unsigned char stack_pointer;
} cpu_6502;

typedef unsigned int instruction_t;

typedef union _operand_t {
    unsigned short address;
    unsigned char byte[2];
} operand_t;

typedef void (*operation_t)(cpu_6502*, operand_t*);

cpu_6502 *create_6502(memory_map_t *memory_map);

void destroy_6502(cpu_6502 *cpu);

int _6502_execute(cpu_6502 *cpu);

void _6502_reset(cpu_6502 *cpu);

#endif // _6502_H