#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "capacitive_touch.pio.h"

// Configuration
#define NUM_KEYS 4
#define FIRST_KEY_PIN 0

// DMA control block structure
typedef struct {
    void* read_addr;
    void* write_addr;
    uint32_t transfer_count;
    uint32_t ctrl;  // The DMA control register value
} dma_control_block_t;

static void capacitive_touch_one_time_setup(PIO pio, uint first_pin, uint num_keys, uint pio_program_offset) {
    // Set up all pins:
    //  - Disable pull-up/pull-down 
    //  - attach to PIO
    //  - set to input mode (high-Z) so they don't interfere with each other when we're not scanning them
    for (uint i = 0; i < num_keys; i++) {
        uint pin = first_pin + i;
        gpio_set_pulls(pin, false, false);  // Disable internal pulls (relying on external pull-up)
        pio_gpio_init(pio, pin);
    }
}

// Set up DMA control blocks for capacitive touch scanning
// Returns pointer to control blocks array (allocated on heap)
// For each key, we need to:
//   1. Write execctrl register
//   2. Write pinctrl register  
//   3. Write restart instruction to kick off measurement
//   4. Read result from RX FIFO
static dma_control_block_t* setup_capacitive_scanner_dma(
    uint dma_worker_chan,
    uint dma_control_chan,
    PIO pio,
    uint sm,
    uint first_pin,
    uint num_keys,
    uint pio_program_offset,
    uint32_t* execctrls,
    uint32_t* pinctrls,
    uint32_t* readings) {
    
    // We need 4 operations per key, plus 1 null control block to signal completion
    uint num_control_blocks = num_keys * 4 + 1;
    dma_control_block_t* control_blocks = malloc(num_control_blocks * sizeof(dma_control_block_t));
    
    if (!control_blocks) {
        printf("ERROR: Failed to allocate control blocks!\n");
        return NULL;
    }
    
    // Prepare the restart instruction (jump to start of program)
    static uint32_t restart_jmp;
    restart_jmp = pio_encode_jmp(pio_program_offset);
    
    // Get worker DMA register base address
    volatile uint32_t* worker_regs = (volatile uint32_t*)&dma_hw->ch[dma_worker_chan];
    
    // Configure base channel configs for each operation type
    // All operations chain to the control DMA
    
    // Config for writing PIO registers (execctrl, pinctrl, instr)
    dma_channel_config pio_write_cfg = dma_channel_get_default_config(dma_worker_chan);
    channel_config_set_transfer_data_size(&pio_write_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&pio_write_cfg, false);
    channel_config_set_write_increment(&pio_write_cfg, false);
    channel_config_set_chain_to(&pio_write_cfg, dma_control_chan);
    // Raise IRQ when 0 is written to trigger register (null control block)
    channel_config_set_irq_quiet(&pio_write_cfg, true);
    
    // Config for reading from PIO RX FIFO (paced by DREQ)
    dma_channel_config pio_read_cfg = dma_channel_get_default_config(dma_worker_chan);
    channel_config_set_transfer_data_size(&pio_read_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&pio_read_cfg, false);
    channel_config_set_write_increment(&pio_read_cfg, false);
    channel_config_set_dreq(&pio_read_cfg, pio_get_dreq(pio, sm, false));
    channel_config_set_chain_to(&pio_read_cfg, dma_control_chan);
    channel_config_set_irq_quiet(&pio_read_cfg, true);
    
    // The DMA peripheral has 4 registers, and they are aliased in 4 different arrangements.
    // The last register in each arrangement is the "trigger" - when you write to it, it starts
    // the transfer using the trigger values plus the then-current values of the other registers.

    //                        +0x0       +0x4         +0x8         +0xC (Trigger)
    // Alias 0 (no prefix):   READ_ADDR  WRITE_ADDR   TRANS_COUNT  CTRL
    // Alias 1 (al1_ prefix): CTRL       READ_ADDR    WRITE_ADDR   TRANS_COUNT
    // Alias 2 (al2_ prefix): CTRL       TRANS_COUNT  READ_ADDR    WRITE_ADDR
    // Alias 3 (al3_ prefix): CTRL       WRITE_ADDR   TRANS_COUNT  READ_ADDR

    // Build control blocks for each key
    for (uint i = 0; i < num_keys; i++) {
        uint block_base = i * 4;
        
        // 1. Write execctrl register
        control_blocks[block_base + 0].read_addr = &execctrls[i];
        control_blocks[block_base + 0].write_addr = (void*)&pio->sm[sm].execctrl;
        control_blocks[block_base + 0].transfer_count = 1;
        control_blocks[block_base + 0].ctrl = pio_write_cfg.ctrl;
        
        // 2. Write pinctrl register
        control_blocks[block_base + 1].read_addr = &pinctrls[i];
        control_blocks[block_base + 1].write_addr = (void*)&pio->sm[sm].pinctrl;
        control_blocks[block_base + 1].transfer_count = 1;
        control_blocks[block_base + 1].ctrl = pio_write_cfg.ctrl;
        
        // 3. Write restart instruction to kick off measurement
        control_blocks[block_base + 2].read_addr = &restart_jmp;
        control_blocks[block_base + 2].write_addr = (void*)&pio->sm[sm].instr;
        control_blocks[block_base + 2].transfer_count = 1;
        control_blocks[block_base + 2].ctrl = pio_write_cfg.ctrl;
        
        // 4. Read result from RX FIFO (paced by PIO)
        control_blocks[block_base + 3].read_addr = (void*)&pio->rxf[sm];
        control_blocks[block_base + 3].write_addr = &readings[i];
        control_blocks[block_base + 3].transfer_count = 1;
        control_blocks[block_base + 3].ctrl = pio_read_cfg.ctrl;
    }
    
    // Add a null control block at the end to signal completion via IRQ
    // When transfer_count is 0, it triggers the IRQ flag
    control_blocks[num_keys * 4].read_addr = NULL;
    control_blocks[num_keys * 4].write_addr = NULL;
    control_blocks[num_keys * 4].transfer_count = 0;
    control_blocks[num_keys * 4].ctrl = 0;  // null trigger ends the chain and raises IRQ
    
    // Configure control DMA to write control blocks to worker DMA registers
    dma_channel_config control_cfg = dma_channel_get_default_config(dma_control_chan);
    channel_config_set_transfer_data_size(&control_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&control_cfg, true);   // Step through control blocks
    channel_config_set_write_increment(&control_cfg, true);  // Step through worker registers
    channel_config_set_ring(&control_cfg, true, 4);          // Ring on write: 16 bytes = 4 registers
    
    dma_channel_configure(dma_control_chan, &control_cfg,
                         worker_regs,           // Write to worker DMA registers
                         control_blocks,        // Read from control blocks
                         4,                     // Transfer 4 words (1 control block), auto-reloads on chain
                         false);                // Don't start yet
    
    printf("Created %u control blocks for %u keys\n", num_control_blocks, num_keys);
    printf("  Control DMA chan %u: writes control blocks to worker DMA\n", dma_control_chan);
    printf("  Worker DMA chan %u: executes scripted operations\n", dma_worker_chan);
    
    return control_blocks;
}

int main() {
    // Initialize stdio for printf
    stdio_init_all();
    
    printf("\n=== RP2040 Capacitive Touch Scanner ===\n");
    printf("System clock: %lu Hz\n", clock_get_hz(clk_sys));
    printf("Initializing %d touch sensors on GPIOs %d-%d\n\n", 
           NUM_KEYS, FIRST_KEY_PIN, FIRST_KEY_PIN + NUM_KEYS - 1);
    
    // Small delay to let serial connection stabilize
    sleep_ms(1000);

    // =========== PIO SETUP ===========
    // Just grabbing PIO0 for now
    PIO pio = pio0;
    uint pio_program_offset = pio_add_program(pio, &capacitive_touch_program);
    
    // Try to claim a state machine
    int sm = pio_claim_unused_sm(pio, true); // Will panic if not available

    printf("Initialized PIO%d SM%d with program at offset %d\n", pio_get_index(pio), sm, pio_program_offset);

    capacitive_touch_one_time_setup(pio, FIRST_KEY_PIN, NUM_KEYS, pio_program_offset);
    
    // Create an array of control structures for each key, so we can switch keys using DMA
    // The default settings for the PIO state machine (independent of which pin is 'hot')
    pio_sm_config c = capacitive_touch_program_get_default_config(pio_program_offset);

    // Set clock divider to ~1 MHz (125 MHz / 125 = 1 MHz)
    // This makes each PIO cycle = 1 microsecond
    sm_config_set_clkdiv(&c, 125.0f);
    
    // Configure autopush: we're using manual push, so disable autopush
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_out_shift(&c, false, false, 32);

    uint32_t execctrls[NUM_KEYS + 1]; // +1 for the null at the end which stops the DMA chain
    uint32_t pinctrls[NUM_KEYS];
    for (uint i = 0; i < NUM_KEYS; i++) {
        uint pin = FIRST_KEY_PIN + i;
        sm_config_set_set_pins(&c, pin, 1);
        sm_config_set_jmp_pin(&c, pin);
        
        // set current pin direction to output so we can drive it low to discharge the sensor
        // TODO: doesn't the PIO program do this too? is it needed here?
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);

        execctrls[i] = c.execctrl;
        pinctrls[i] = c.pinctrl;
    }
    execctrls[NUM_KEYS] = 0; // null config at the end to stop the DMA chain after we've gone through all keys
    
    // Initialize the state machine with the first config, starting at wait_for_restart
    sm_config_set_set_pins(&c, FIRST_KEY_PIN, 1);
    sm_config_set_jmp_pin(&c, FIRST_KEY_PIN);
    pio_sm_init(pio, sm, pio_program_offset + capacitive_touch_offset_wait_for_restart, &c);
    
    // Start the state machine for the first time (it'll be in wait_for_restart, waiting for us to kick it off by writing to execctrl/pinctrl)
    pio_sm_set_enabled(pio, sm, true);

    // =========== DMA SETUP ===========
    
    // Allocate readings array
    static uint32_t readings[NUM_KEYS];
    
    // Claim DMA channels - only need 2 with control block approach!
    uint dma_worker_chan = dma_claim_unused_channel(true);
    uint dma_control_chan = dma_claim_unused_channel(true);
    
    printf("Using worker DMA channel %u\n", dma_worker_chan);
    printf("Using control DMA channel %u\n\n", dma_control_chan);
    
    // Build control blocks for the complete scan sequence
    dma_control_block_t* control_blocks = setup_capacitive_scanner_dma(
        dma_worker_chan,
        dma_control_chan,
        pio,
        sm,
        FIRST_KEY_PIN,
        NUM_KEYS,
        pio_program_offset,
        execctrls,
        pinctrls,
        readings);
    
    if (!control_blocks) {
        printf("Failed to set up DMA!\n");
        return 1;
    }
    
    printf("\nStarting continuous scanning...\n\n");
    
    // Main loop: continuously scan and display touch sensor values
    while (true) {
        // Start the control DMA to begin the scan sequence
        dma_channel_start(dma_control_chan);
        
        // Wait for worker DMA to assert its IRQ flag when it hits the null control block
        // This indicates the scan is complete
        while (!(dma_hw->intr & (1u << dma_worker_chan))) {
            tight_loop_contents();
        }
        
        // Clear the IRQ flag
        dma_hw->ints0 = 1u << dma_worker_chan;
        
        // Reset control DMA read address to start of array for next scan
        dma_channel_set_read_addr(dma_control_chan, control_blocks, false);
        
        // Print all readings on one line
        printf("Keys: ");
        for (uint i = 0; i < NUM_KEYS; i++) {
            printf("K%d:%5lu  ", i, 0xFFFFFFFF - readings[i]);
        }
        printf("\n");
        
        // Small delay to make output readable
        sleep_ms(100);
    }
    
    return 0;
}
