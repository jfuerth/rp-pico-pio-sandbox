#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "capacitive_touch.pio.h"

// Configuration
#define NUM_KEYS 4
#define FIRST_KEY_PIN 0

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

    
    // DMA to store readings coming off the PIO
    // (from the RX FIFO of the state machine, paced by the state machine's DREQ)
    uint pio_rx_dma_chan = dma_claim_unused_channel(true);
    static uint32_t readings[NUM_KEYS];

    uint pio_execctrl_dma_chan = dma_claim_unused_channel(true); // execctrls -> pio->sm[sm].execctrl
    uint pio_pinctrl_dma_chan = dma_claim_unused_channel(true);  // pinctrls -> pio->sm[sm].pinctrl
    uint pio_restart_dma_chan = dma_claim_unused_channel(true);  // restart_jmp -> pio->sm[sm].instr (to kick off each measurement by jumping to the start of the program)

    // Configure pio_rx_dma_chan
    dma_channel_config dma_cfg = dma_channel_get_default_config(pio_rx_dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true); // step to the next entry in the readings buffer after each transfer
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio, sm, false)); // wait on DREQ for the RX FIFO of our state machine
    channel_config_set_chain_to(&dma_cfg, pio_execctrl_dma_chan); // chain to execctrl DMA channel for continuous transfers

    // Configure pio_execctrl_dma_chan
    dma_channel_config pio_execctrl_dma_cfg = dma_channel_get_default_config(pio_execctrl_dma_chan);
    channel_config_set_transfer_data_size(&pio_execctrl_dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&pio_execctrl_dma_cfg, true); // step through the execctrls array
    channel_config_set_write_increment(&pio_execctrl_dma_cfg, false); // always write to the same place (execctrl register)
    channel_config_set_chain_to(&pio_execctrl_dma_cfg, pio_pinctrl_dma_chan); // chain to pinctrl DMA channel after each transfer

    // Configure pio_pinctrl_dma_chan
    dma_channel_config pio_pinctrl_dma_cfg = dma_channel_get_default_config(pio_pinctrl_dma_chan);
    channel_config_set_transfer_data_size(&pio_pinctrl_dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&pio_pinctrl_dma_cfg, true); // step through the pinctrls array
    channel_config_set_write_increment(&pio_pinctrl_dma_cfg, false); // always write to the same place (pinctrl register)
    channel_config_set_chain_to(&pio_pinctrl_dma_cfg, pio_restart_dma_chan); // chain to restart DMA channel after each transfer

    // Configure pio_restart_dma_chan
    uint32_t restart_jmp = pio_encode_jmp(pio_program_offset); // the instruction we want to write to the instr register to kick off each measurement by jumping to the start of the program
    dma_channel_config pio_restart_dma_cfg = dma_channel_get_default_config(pio_restart_dma_chan);
    channel_config_set_transfer_data_size(&pio_restart_dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&pio_restart_dma_cfg, false); // always read the same value (the restart_jmp instruction)
    channel_config_set_write_increment(&pio_restart_dma_cfg, false); // always write to the same place (instr register)
    channel_config_set_chain_to(&pio_restart_dma_cfg, pio_rx_dma_chan); // chain back to the first DMA channel to create a continuous loop

    printf("\nStarting continuous scanning...\n\n");
    
    // Main loop: continuously read and display touch sensor values
    while (true) {

        dma_channel_configure(pio_rx_dma_chan, &dma_cfg,
                            readings, // write address
                            &pio->rxf[sm], // read address (RX FIFO of our state machine)
                            1, // transfer count (we need to update the PIO config for each key, so we'll do one transfer at a time)
                            false); // don't start yet, we'll trigger manually
        dma_channel_configure(pio_execctrl_dma_chan, &pio_execctrl_dma_cfg,
                            &pio->sm[sm].execctrl, // write address (execctrl register of our state machine)
                            execctrls, // read address (our array of execctrl values for each key)
                            1, // transfer count (transfer one value per key measurement)
                            false); // don't start yet; will be triggered by the pio_rx_dma_chan
        dma_channel_configure(pio_pinctrl_dma_chan, &pio_pinctrl_dma_cfg,
                            &pio->sm[sm].pinctrl, // write address (pinctrl register of our state machine)
                            pinctrls, // read address (our array of pinctrl values for each key)
                            1, // transfer count (transfer one value per key measurement)
                            false); // don't start yet; will be triggered by the pio_execctrl_dma_chan
        dma_channel_configure(pio_restart_dma_chan, &pio_restart_dma_cfg,
                            &pio->sm[sm].instr, // write address (instr register of our state machine)
                            &restart_jmp, // read address (the instruction to jump to the start of the program)
                            1, // transfer count (just one instruction)
                            false); // don't start yet; will be triggered by the pio_pinctrl_dma_chan

        // Read from all sensor FIFOs
        dma_channel_start(pio_execctrl_dma_chan); // trigger the DMA transfer to read the result of this measurement when it's ready
            
        // Wait for data to be available in FIFO
        printf("Waiting for data...");
        dma_channel_wait_for_finish_blocking(pio_rx_dma_chan);            
        printf("DMA completed. Reading values...\n");

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
