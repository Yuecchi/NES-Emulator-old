#include <stdio.h>

#include "nes.h"

int main() {
    nes_t *nes = create_nes();
    nes_load_rom(nes, "smb.nes");
    nes_run(nes);
return 0;
}