#ifndef NES_H
#define NES_H

#include "6502.h"

typedef struct _nes_t {
    memory_map_t memory_map;                    //
    unsigned char *ram; //[0x800];              // 2KB of general purpose memory
    unsigned char *ppu_registers; //[0x8];      // registers which the cpu and ppu communicate via
    unsigned char *io_registers; //[0x20];      // apu registers / controller ports   
    unsigned char *expansion_rom; //[0x1FE0];   //
    unsigned char *sram; //[0x2000]             //
    unsigned char *prg_rom_lower_bank;          //
    unsigned char *prg_rom_upper_bank;          //
    unsigned char rom[0x100000];                //
    cpu_6502 *cpu;
} nes_t;

nes_t *create_nes();

void destroy_nes(nes_t *nes);

void nes_load_rom(nes_t *nes, const char *rom_src);

void nes_run(nes_t *nes);

#endif // NES_H 

/* notes
 *
 * On system power up
 * 
 * - status register = 0x34 (00110100) which sets flag I, disabling interrupt requests
 * - registers A, X and Y = 0x0
 * - stack pointer = 0xfd
 * - location 0x4017 = 0x0
 * - location 0x4015 = 0x0
 * - locations 0x4000 to 0x400f = 0x0
 * - locations 0x4010 to 0x4013 = 0x0
 * - locations 0x0000 to 0x07ff = 0x0
 */