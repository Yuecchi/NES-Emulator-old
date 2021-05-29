#include <stdlib.h>
#include <stdio.h>

#include "memory_map.h"

memory_map_t *create_memory_map(unsigned int capacity) {
    memory_map_t *map = malloc(sizeof(memory_map_t));
    map->buckets = calloc(capacity, sizeof(mm_node_t*));
    map->current_bucket = 0x0;
    return map;
}

void mm_add_node(memory_map_t *map, unsigned char **data, unsigned int size) {
    // create a new memory map node and assign its start
    // and end location. The start end location will 
    // indicate the address range this node will occupy 
    // within the memory map
    mm_node_t *node = malloc(sizeof(mm_node_t));
    node->start = map->current_bucket;
    node->end = node->start + size;
    // assign references to the node from the map's buckets
    // according to the address range which the node occupies
    for (int i = node->start; i < node->end; i += 1) {
        map->buckets[i] = node;
    }
    map->current_bucket = node->end;
    node->data = data;
}

unsigned char mm_read(memory_map_t *map, unsigned int address) {
    mm_node_t node = *(map->buckets[address]);
    return (*node.data)[address - node.start];
}

void mm_write(memory_map_t *map, unsigned int address, unsigned char data) {
    mm_node_t node = *(map->buckets[address]);
    (*node.data)[address - node.start] = data;
}
