#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

typedef struct _mm_node_t *mm_node_t;

typedef struct _memory_map_t {
    mm_node_t head;
    mm_node_t tail;
} memory_map_t;

void mm_add_node(memory_map_t *map, unsigned char **data, unsigned int size);

unsigned char mm_read(memory_map_t *map, unsigned int address);

void mm_write(memory_map_t *map, unsigned int address, unsigned char data);

// debug
void mm_print(memory_map_t *map);

#endif