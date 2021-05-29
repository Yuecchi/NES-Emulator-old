#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

typedef struct _mm_node_t {
    unsigned int start;
    unsigned int end;
    unsigned char **data;
} mm_node_t;

typedef struct _memory_map_t {
    mm_node_t **buckets;
    unsigned int current_bucket;
} memory_map_t;

memory_map_t *create_memory_map(unsigned int capacity);

void mm_add_node(memory_map_t *map, unsigned char **data, unsigned int size);

unsigned char mm_read(memory_map_t *map, unsigned int address);

void mm_write(memory_map_t *map, unsigned int address, unsigned char data);

#endif