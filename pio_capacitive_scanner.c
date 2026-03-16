#include <stdio.h>
#include "pico/stdlib.h"
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

static inline void capacitive_touch_config_set_pin(pio_sm_config *c, PIO pio, uint sm, uint offset, uint pin) {
    // Configure the pin for SET operations
    sm_config_set_set_pins(c, pin, 1);
    
    // Configure the pin for JMP PIN test
    sm_config_set_jmp_pin(c, pin);
    
    // set current pin direction to output so we can drive it low to discharge the sensor
    // TODO: doesn't the PIO program do this too? is it needed here?
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
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

    pio_sm_config configs[NUM_KEYS];
    for (uint i = 0; i < NUM_KEYS; i++) {
        configs[i] = c;
        capacitive_touch_config_set_pin(&configs[i], pio, sm, pio_program_offset, FIRST_KEY_PIN + i);
    }
    
    // Initialize the state machine with the first config, starting at wait_for_restart
    pio_sm_init(pio, sm, pio_program_offset + capacitive_touch_offset_wait_for_restart, &configs[0]);
    
    // Start the state machine for the first time (it'll be in wait_for_restart, waiting for us to kick it off by writing to execctrl/pinctrl)
    pio_sm_set_enabled(pio, sm, true);

    printf("\nStarting continuous scanning...\n\n");
    
    // Array to store readings
    uint32_t readings[NUM_KEYS];
    
    // Main loop: continuously read and display touch sensor values
    while (true) {
        // Read from all sensor FIFOs
        for (uint i = 0; i < NUM_KEYS; i++) {
            printf("\nReading key %d on pin %d...\n", i, FIRST_KEY_PIN + i);

            // Write new config directly to registers (DMA-able operation!)
            // The SM is safely stuck in wait_for_restart loop at this point
            pio->sm[sm].execctrl = configs[i].execctrl;
            pio->sm[sm].pinctrl = configs[i].pinctrl;
            // Note: clkdiv and shiftctrl don't change between configs, so we skip them
            
            // Jump to start to begin the measurement
            pio_sm_exec(pio, sm, pio_encode_jmp(pio_program_offset));
            
            printf("Waiting for data...");

            // Wait for data to be available in FIFO
            while (pio_sm_is_rx_fifo_empty(pio, sm)) {
                tight_loop_contents();
            }
            
            printf("Data available! Reading... ");
            // Read the cycle count from FIFO
            // PIO counts down from 0xFFFFFFFF, so actual count = MAX - reading
            uint32_t countdown_value = pio_sm_get_blocking(pio, sm);
            readings[i] = 0xFFFFFFFF - countdown_value;
            printf("Done! Count: %lu\n", readings[i]);
        }
        
        // Print all readings on one line
        printf("Keys: ");
        for (uint i = 0; i < NUM_KEYS; i++) {
            printf("K%d:%5lu  ", i, readings[i]);
        }
        printf("\n");
        
        // Small delay to make output readable
        // You can reduce or remove this for faster scanning
        sleep_ms(100);
    }
    
    return 0;
}
