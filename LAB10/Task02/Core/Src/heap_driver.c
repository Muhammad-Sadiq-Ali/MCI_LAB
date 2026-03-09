#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

/// Allocation bitmap: 0 = free, 1 = used
uint8_t block_map[BLOCK_COUNT];  // fixed: was [BLOCK_SIZE], should be [BLOCK_COUNT]

void heap_init() {
    for (int i = 0; i < BLOCK_COUNT; i++){
        block_map[i] = 0;               
    }
}

void* heap_alloc(size_t size) { 
    int blocksNeeded = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    int consecutiveFree = 0;

    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (block_map[i] == 0) {
            consecutiveFree++;
            if (consecutiveFree == blocksNeeded) {
                int start_block = i - blocksNeeded + 1;
                for (int j = start_block; j <= i; j++) {
                    block_map[j] = 1;
                }
                return (void*)(HEAP_START_ADDR + start_block * BLOCK_SIZE);
            }
        } else {
            consecutiveFree = 0;
        }
    }
    return NULL;
}

void heap_free(void* ptr) {
    if (ptr == NULL) return;

    int blockIndex = ((uint8_t*)ptr - HEAP_START_ADDR) / BLOCK_SIZE;
    for (int i = blockIndex; i < BLOCK_COUNT; i++) {
        if (block_map[i] == 1) {
            block_map[i] = 0;
        } else {
            break;
        }
    }
}