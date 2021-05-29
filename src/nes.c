#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nes.h"

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
    
    nes->ram = calloc(1, 0x2000);
    for (int i = 0x0000; i < 0x2000; i += 0x2000) {
        mm_add_node(nes->memory_map, &nes->ram, 0x2000);
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

    // assign the reset vector for the cpu. This method
    // is only correct for rom sizes between 16 and 32KB.
    // The follow must be done in order to have it work 
    // for a wider variety of cases:
    //
    // - need to check if the trainer is present
    // - need to check how many 16KB banks the rom contains
    //
    nes->prg_rom_lower_bank = &nes->rom[0x10];
    nes->prg_rom_upper_bank = &nes->rom[0x4010];
    unsigned short reset_vector = (nes->prg_rom_upper_bank[0x3ffd] * 0x100) + nes->prg_rom_upper_bank[0x3ffc];
    _6502_set_pc(nes->cpu, reset_vector);
}

void nes_run(nes_t *nes) {

    // force system to always be in vblank
    mm_write(nes->memory_map, 0x2002, 0x80);

    _6502_execute(nes->cpu); // SEI       (disable interrupt requests)
    _6502_execute(nes->cpu); // CLD       (disable decimal mode)
    _6502_execute(nes->cpu); // LDA #$10
    _6502_execute(nes->cpu); // STA $2000 (disable NMI)
    _6502_execute(nes->cpu); // LDX #$FF
    _6502_execute(nes->cpu); // TSX       (set up the stack)
    _6502_execute(nes->cpu); // LDA $2002
    _6502_execute(nes->cpu); // BPL $FB   (first vblank wait)
    _6502_execute(nes->cpu); // LDA $2002
    _6502_execute(nes->cpu); // BPL $FB   (second vblank wait)
    _6502_execute(nes->cpu); // LDY #$FE
    _6502_execute(nes->cpu); // LDX #$05
    _6502_execute(nes->cpu); // LDA $07D7, X
    _6502_execute(nes->cpu); // CMP #$0A
}
