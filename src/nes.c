#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nes.h"

#define NMI_ENABLED (nes->ppu_registers[0] & 0x80)
#define VBLANK_SIGNALLED (nes->ppu_registers[2] & 0x80)

#define DOTS_PER_SCANLINE 341
#define DOTS_PER_CPU_CYCLE 3

nes_t *create_nes() {
    nes_t *nes = calloc(1, sizeof(nes_t));
    nes->memory_map = create_memory_map(0x10000);
    /* Add system RAM to memory map
     * in the following locations:
     * 
     * 0x0000 ----------
     *        |        |
     *        |  RAM   |
     *        |        |
     * 0x0800 ----------
     *        |  FIRST |
     *        |   RAM  |
     *        | MIRROR |
     * 0x1000 ----------
     *        | SECOND |
     *        |   RAM  |
     *        | MIRROR |
     * 0x1800 ----------
     *        |  THIRD |
     *        |   RAM  |
     *        | MIRROR |
     * 0x2000 ----------
     *
     */
    nes->ram = calloc(1, 0x800);
    for (int i = 0x0; i < 0x2000; i += 0x800) {
        mm_add_node(nes->memory_map, &nes->ram, 0x800);
    }

    /*
     * Add ppu io registers to the memory map
     * from 0x2000 to 0x2007, and mirror these
     * registers every 8 bytes from 0x2008 to
     * 0x3fff
     */
    nes->ppu_registers = calloc(1, 0x8);
    for (int i = 0x2000; i < 0x4000; i += 0x8) {
        mm_add_node(nes->memory_map, &nes->ppu_registers, 0x8);
    }

    /*
     * Add apu registers and io ports for joypads
     * to the memory map from 0x4000 to 0x4020 
     */

    nes->io_registers = calloc(1, 0x20);
    mm_add_node(nes->memory_map, &nes->io_registers, 0x20);

    /*
     *
     */
    nes->expansion_rom = calloc(1, 0x1fe0);
    mm_add_node(nes->memory_map, &nes->expansion_rom, 0x1fe0);

    /*
     *
     */
    nes->sram = calloc(1, 0x2000);
    mm_add_node(nes->memory_map, &nes->sram, 0x2000);
    
    /*
     *
     */
    mm_add_node(nes->memory_map, &nes->prg_rom_lower_bank, 0x4000);
    mm_add_node(nes->memory_map, &nes->prg_rom_upper_bank, 0x4000);

    // create the cpu
    nes->cpu = create_6502(nes->memory_map);

    return nes; 
}

void destroy_nes(nes_t *nes) {
    destroy_6502(nes->cpu);
    free(nes);
}

void nes_load_rom(nes_t *nes, const char *nes_src) {
    FILE *rom_src = fopen("smb.nes", "rb");
    if (!rom_src) {
        printf("could not find rom file\n");
    } else {
        fread(nes->rom, sizeof(unsigned char), sizeof(nes->rom), rom_src);
    }
    fclose(rom_src);

    // Attach the program rom banks to the memory map. This 
    // is only correct for rom sizes between 16 and 32KB.
    // The follow must be done in order to have it work 
    // for a wider variety of cases:
    //
    // - need to check if the trainer is present
    // - need to check how many 16KB banks the rom contains
    //
    nes->prg_rom_lower_bank = &nes->rom[0x10];
    nes->prg_rom_upper_bank = &nes->rom[0x4010];
    _6502_reset(nes->cpu);   
}

void nes_run(nes_t *nes) {
    int cycles;
    int dots_this_frame = 0;
    int in_vblank       = 0;
    
    while (1) {
       
        // execute the next instruction 
        cycles = 0;
        cycles += _6502_execute(nes->cpu);

        // when vblank is signalled by the ppu, perform a
        // non-masking interrupt and clear bit 7 of PPUSTATUS
        if (VBLANK_SIGNALLED) {
            nes->ppu_registers[2] &= 0x7f;
            if (NMI_ENABLED) {
                cycles += _6502_nmi(nes->cpu);
            } 
        }

        // use the number of cycles the instruction 
        // required to execute to calculate the number)
        // of ppu cycles (dots which occurred during the 
        // execution of the  cpu instruction
        dots_this_frame += (cycles * DOTS_PER_CPU_CYCLE);

        // signal that vblank has started on the 2nd dot
        // of every 242nd scanline
        if (!in_vblank && dots_this_frame >= (241 * DOTS_PER_SCANLINE) + 2) {
            nes->ppu_registers[2] |= 0x80;
            in_vblank = 1;
        }

        // signal that vblank has ended on the 2nd dot
        // every 262nd scanline
        if (dots_this_frame >= (261 * DOTS_PER_SCANLINE) + 2) {
            nes->ppu_registers[2] &= 0x7f;
            in_vblank = 0;
        }

        // reset dots back to 0 every 262 scanlines
        if (dots_this_frame >= (262 * DOTS_PER_SCANLINE)) {
            dots_this_frame = dots_this_frame % (262 * DOTS_PER_SCANLINE);
        }
    }

}
