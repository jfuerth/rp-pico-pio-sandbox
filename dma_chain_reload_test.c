#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"

int main() {
    stdio_init_all();
    sleep_ms(1000);
    
    printf("\n=== DMA Chain Reload Test ===\n");
    printf("Testing if RING setting enables automatic reload on chaining\n\n");
    
    // Test parameter: set to true to enable ring, false to disable
    bool use_ring_buffer = false;  // Change this to test
    
    printf("Configuration: ring buffer %s\n\n", use_ring_buffer ? "ENABLED" : "DISABLED");
    
    // Claim two DMA channels
    uint dma_a = dma_claim_unused_channel(true);
    uint dma_b = dma_claim_unused_channel(true);
    
    printf("Using DMA channel A: %u\n", dma_a);
    printf("Using DMA channel B: %u\n\n", dma_b);
    
    // Source arrays for each DMA - using distinctive hex patterns
    // MUST be aligned to ring buffer size (64 bytes)
    static uint32_t source_a[10] __attribute__((aligned(32))) = {
        0xAAAA0000, 0xAAAA0001, 0xAAAA0002, 0xAAAA0003, 0xAAAA0004, 
        0xAAAA0005, 0xAAAA0006, 0xAAAA0007, 0xAAAA0008, 0xAAAA0009
    };
    static uint32_t source_b[10] __attribute__((aligned(32))) = {
        0xBBBB0000, 0xBBBB0001, 0xBBBB0002, 0xBBBB0003, 0xBBBB0004, 
        0xBBBB0005, 0xBBBB0006, 0xBBBB0007, 0xBBBB0008, 0xBBBB0009
    };
    
    // Single target that both DMAs write to
    static volatile uint32_t target = 0;
    
    // Configure DMA A: reads from source_a, writes to target, chains to B
    dma_channel_config cfg_a = dma_channel_get_default_config(dma_a);
    channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_a, true);     // Increment through source_a
    channel_config_set_write_increment(&cfg_a, false);   // Always write to target
    if (use_ring_buffer) {
        channel_config_set_ring(&cfg_a, false, 5);       // Ring on read: 5 bits (32 bytes = 8 array slots)
    }
    channel_config_set_chain_to(&cfg_a, dma_b);         // Chain to B when done
    
    dma_channel_configure(dma_a, &cfg_a,
                         (void*)&target,      // Write to target
                         source_a,            // Read from source_a
                         1,                   // Transfer 1 item
                         false);              // Don't start yet
    
    // Configure DMA B: reads from source_b, writes to target, chains to A
    dma_channel_config cfg_b = dma_channel_get_default_config(dma_b);
    channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_b, true);     // Increment through source_b
    channel_config_set_write_increment(&cfg_b, false);   // Always write to target
    if (use_ring_buffer) {
        channel_config_set_ring(&cfg_b, false, 5);       // Ring on read: 5 bits (32 bytes = 8 array slots)
    }
    channel_config_set_chain_to(&cfg_b, dma_a);         // Chain to A when done
    
    dma_channel_configure(dma_b, &cfg_b,
                         (void*)&target,      // Write to target
                         source_b,            // Read from source_b
                         1,                   // Transfer 1 item
                         false);              // Don't start yet
    
    printf("DMA A configured: source_a -> target, chain to B, count=1, ring=%s\n", use_ring_buffer ? "ON" : "OFF");
    printf("DMA B configured: source_b -> target, chain to A, count=1, ring=%s\n\n", use_ring_buffer ? "ON" : "OFF");
    
    // Record target values over time
    printf("Starting DMA A...\n\n");
    dma_channel_start(dma_a);
    
    printf("Time | DMA A Count | DMA B Count | Target Value\n");
    printf("-----+-------------+-------------+--------------\n");
    
    for (int i = 0; i < 20; i++) {
        uint32_t count_a = 99; //dma_hw->ch[dma_a].transfer_count;
        uint32_t count_b = 99; //dma_hw->ch[dma_b].transfer_count;
        uint32_t val = target;
        
        printf("%4d | %11lu | %11lu | 0x%08lx\n", i, count_a, count_b, val);
        //sleep_us(777);
        //sleep_ms(1);
    }
    
    printf("\n=== Test Complete ===\n");
    printf("If chaining reloads transfer_count, we should see alternating\n");
    printf("values: 0xAAAA0000, 0xBBBB0000, 0xAAAA0001, 0xBBBB0001, etc.\n");
    printf("If not, everything will stop after first two transfers.\n");
    
    for (;;) {
        tight_loop_contents();
    }
    return 0;
}
