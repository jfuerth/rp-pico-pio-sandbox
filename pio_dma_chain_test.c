#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pio_count.pio.h"

int main() {
    stdio_init_all();
    sleep_ms(1000);
    
    printf("\n=== PIO Test with DMA Control Blocks (Phase 6) ===\n");
    
    // Set up PIO
    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, true);
    
    // Load program
    uint offset = pio_add_program(pio, &pio_count_program);
    printf("Loaded PIO program at offset %d\n", offset);
    
    // Initialize state machine with default config
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_clkdiv(&c, 10.0f);  // Slow clock for observation
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    
    printf("SM%d enabled\n", sm);
    
    // Claim DMA channels - only need 2 now!
    uint dma_worker_chan = dma_claim_unused_channel(true);
    uint dma_control_chan = dma_claim_unused_channel(true);
    
    printf("Using worker DMA channel %u\n", dma_worker_chan);
    printf("Using control DMA channel %u\n\n", dma_control_chan);
    
    // Data arrays
    static uint32_t values_to_send[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    static volatile uint32_t results[10];
    
    // Clear results
    for (int i = 0; i < 10; i++) {
        results[i] = 99;
    }
    
    // Create control blocks - these are the configurations for the worker DMA
    // We need 20 control blocks: 10 for TX, 10 for RX, alternating
    typedef struct {
        void* read_addr;
        void* write_addr;
        uint32_t transfer_count;
        uint32_t ctrl;  // The DMA control register value
    } dma_control_block_t;
    
    static dma_control_block_t control_blocks[20];
    
    // Configure the control register values for TX and RX operations
    // Both will chain directly to control DMA (which will auto-reload its count=4)
    dma_channel_config tx_cfg = dma_channel_get_default_config(dma_worker_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&tx_cfg, false);   // Don't increment - control block will set address
    channel_config_set_write_increment(&tx_cfg, false);  // Don't increment - always write to TX FIFO
    channel_config_set_chain_to(&tx_cfg, dma_control_chan);  // Chain directly to control DMA
    
    dma_channel_config rx_cfg = dma_channel_get_default_config(dma_worker_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&rx_cfg, false);   // Always read from RX FIFO
    channel_config_set_write_increment(&rx_cfg, false);  // Don't increment - control block will set address
    channel_config_set_dreq(&rx_cfg, pio_get_dreq(pio, sm, false));  // Pace on PIO RX FIFO
    channel_config_set_chain_to(&rx_cfg, dma_control_chan);  // Chain directly to control DMA
    
    // Build control blocks: TX then RX for each value
    for (int i = 0; i < 10; i++) {
        // TX control block
        control_blocks[i * 2].read_addr = &values_to_send[i];
        control_blocks[i * 2].write_addr = &pio->txf[sm];
        control_blocks[i * 2].transfer_count = 1;
        control_blocks[i * 2].ctrl = tx_cfg.ctrl;
        
        // RX control block
        control_blocks[i * 2 + 1].read_addr = &pio->rxf[sm];
        control_blocks[i * 2 + 1].write_addr = (void*)&results[i];
        control_blocks[i * 2 + 1].transfer_count = 1;
        control_blocks[i * 2 + 1].ctrl = rx_cfg.ctrl;
    }
    
    printf("Control blocks created: 20 blocks (10 TX + 10 RX pairs)\n");
    
    // Get worker DMA register base address
    volatile uint32_t* worker_regs = (volatile uint32_t*)&dma_hw->ch[dma_worker_chan];
    
    // Print first two control blocks for debugging
    printf("First TX control block (index 0):\n");
    printf("  read_addr:  0x%08lx\n", (uint32_t)control_blocks[0].read_addr);
    printf("  write_addr: 0x%08lx\n", (uint32_t)control_blocks[0].write_addr);
    printf("  count:      %lu\n", control_blocks[0].transfer_count);
    printf("  ctrl:       0x%08lx (chain_to=%lu)\n", control_blocks[0].ctrl, (control_blocks[0].ctrl >> 11) & 0xF);
    
    printf("First RX control block (index 1):\n");
    printf("  read_addr:  0x%08lx\n", (uint32_t)control_blocks[1].read_addr);
    printf("  write_addr: 0x%08lx\n", (uint32_t)control_blocks[1].write_addr);
    printf("  count:      %lu\n", control_blocks[1].transfer_count);
    printf("  ctrl:       0x%08lx (chain_to=%lu)\n", control_blocks[1].ctrl, (control_blocks[1].ctrl >> 11) & 0xF);
    
    printf("Worker DMA register addresses:\n");
    printf("  READ_ADDR:   0x%08lx\n", (uint32_t)&worker_regs[0]);
    printf("  WRITE_ADDR:  0x%08lx\n", (uint32_t)&worker_regs[1]);
    printf("  TRANS_COUNT: 0x%08lx\n", (uint32_t)&worker_regs[2]);
    printf("  CTRL_TRIG:   0x%08lx\n", (uint32_t)&worker_regs[3]);
    printf("\n");
    
    // Configure control DMA to write control blocks to worker DMA registers
    // The worker DMA channel has 4 registers we need to write:
    // - READ_ADDR (offset 0x0)
    // - WRITE_ADDR (offset 0x4)  
    // - TRANS_COUNT (offset 0x8)
    // - CTRL_TRIG (offset 0xC) - writing this starts the transfer
    
    dma_channel_config control_cfg = dma_channel_get_default_config(dma_control_chan);
    channel_config_set_transfer_data_size(&control_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&control_cfg, true);   // Step through control blocks
    channel_config_set_write_increment(&control_cfg, true);  // INCREMENT through worker registers (ring will wrap)
    channel_config_set_ring(&control_cfg, true, 4);          // Ring on write: 16 bytes = 4 registers
    // NO CHAIN from control - it just writes 4 registers and stops
    
    dma_channel_configure(dma_control_chan, &control_cfg,
                         worker_regs,           // Write to worker DMA registers
                         control_blocks,        // Read from control blocks
                         4,                     // Transfer 4 registers (1 control block at a time)
                         false);                // Don't start yet
    
    printf("2-DMA setup complete:\n");
    printf("  Control DMA: writes control blocks (count=4, auto-reloads on chain trigger)\n");
    printf("  Worker DMA: executes operations (count=1, chains back to control)\n");
    printf("\nStarting control DMA to orchestrate worker DMA...\n\n");
    
    // Monitor for some time with detailed stats every 1ms
    printf("Monitoring DMA and PIO state:\n");
    printf("Time | Ctrl Count | Worker Count | RX FIFO Level | TX FIFO Level | Results\n");
    printf("-----+------------+--------------+---------------+---------------+---------\n");

    dma_channel_start(dma_control_chan);
    
    for (int i = 0; i < 10; i++) {
        uint32_t ctrl_count = dma_hw->ch[dma_control_chan].transfer_count;
        uint32_t worker_count = dma_hw->ch[dma_worker_chan].transfer_count;
        
        // Get PIO FIFO levels
        uint32_t rxf_level = pio_sm_get_rx_fifo_level(pio, sm);
        uint32_t txf_level = pio_sm_get_tx_fifo_level(pio, sm);
        
        // Count filled results
        int filled = 0;
        for (int j = 0; j < 10; j++) {
            if (results[j] != 99) filled++;
        }
        
        printf("%4d | %10lu | %12lu | %13lu | %13lu | %d/10\n",
               i, ctrl_count, worker_count, rxf_level, txf_level, filled);
        
        // On first iteration, also show worker DMA registers after control DMA ran
        if (i == 0) {
            printf("     | Worker regs: READ=0x%08lx WRITE=0x%08lx COUNT=%lu CTRL=0x%08lx\n",
                   worker_regs[0], worker_regs[1], worker_regs[2], worker_regs[3]);
        }
    }
    
    printf("\nAfter 10 observations:\n");
    
    printf("Final results: ");
    for (int i = 0; i < 10; i++) {
        printf("%lu ", results[i]);
    }

    printf("\n=== Test Complete ===\n");
    
    for (;;) {
        tight_loop_contents();
    }
    return 0;
}
