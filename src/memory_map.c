#include <stdlib.h>
#include <stdio.h>

#include "memory_map.h"

struct _mm_node_t {
    unsigned int start;
    unsigned int end;
    unsigned char **data;
    mm_node_t next;
};

mm_node_t create_node() {
    mm_node_t node;
    node = malloc(sizeof(*node));
    return node;
}

void mm_add_node(memory_map_t *map, unsigned char **data, unsigned int size) {
    if (!map->head) {
        map->head = create_node();
        map->head->data  = data;
        map->head->start = 0x0;
        map->head->end   = map->head->start + size;
        map->head->next  = 0x0; // null 
        map->tail        = map->head;   
        return;
    }

    mm_node_t node = map->tail;  
    node->next = create_node();
    node->next->data  = data;
    node->next->start = node->end;
    node->next->end   = node->next->start + size;
    node->next->next  = 0x0; // null
    map->tail = node->next;
}

unsigned char mm_read(memory_map_t *map, unsigned int address) {
    mm_node_t node = map->head;
    while (address >= node->end) {
        node = node->next;
    }
    return (*node->data)[address - node->start];
}

void mm_write(memory_map_t *map, unsigned int address, unsigned char data) {
    mm_node_t node = map->head;
    while (address >= node->end) {
        node = node->next;
    }
    (*node->data)[address - node->start] = data;
}

void mm_print(memory_map_t *map) {
    unsigned int node_count = 0;
    mm_node_t node = map->head;
    while (node) {
        printf("%i: start: 0x%04x,  end: 0x%04x\n", node_count, node->start, node->end);
        node = node->next;
        node_count++;
    }
}
