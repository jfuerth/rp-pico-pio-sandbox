#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "capacitive_touch.pio.h"

// Configuration
#define NUM_KEYS 4
#define FIRST_KEY_PIN 0

static inline void capacitive_touch_program_set_pin(PIO pio, uint sm, uint offset, uint pin) {
    // Get default state machine config
    pio_sm_config c = capacitive_touch_program_get_default_config(offset); // TODO pass in a config to modify (we don't need to reset all these things every time if we're just changing the pin)
    
    // Configure the pin for SET operations
    sm_config_set_set_pins(&c, pin, 1);
    
    // Configure the pin for JMP PIN test
    sm_config_set_jmp_pin(&c, pin);
    
    // Set clock divider to ~1 MHz (125 MHz / 125 = 1 MHz)
    // This makes each PIO cycle = 1 microsecond
    sm_config_set_clkdiv(&c, 125.0f);
    
    // Configure autopush: we're using manual push, so disable autopush
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_out_shift(&c, false, false, 32);

    // Set the pin to input mode initially (high-Z) and give control to PIO
    gpio_set_pulls(pin, false, false);  // Disable internal pulls (relying on external pull-up)
    pio_gpio_init(pio, pin); // TODO we should only need to do this once per pin, not every time we change the pin for the program
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);

    // apply the config and set the initial PC to the start of our program (halts the PIO SM)
    pio_sm_init(pio, sm, offset, &c);
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
    // Each sensor gets its own state machine
    PIO pio = pio0;
    uint pio_program_offset = pio_add_program(pio, &capacitive_touch_program);
    
    // Try to claim a state machine
    int sm = pio_claim_unused_sm(pio, true); // Will panic if not available

    printf("Initialized PIO%d SM%d with program at offset %d\n", pio_get_index(pio), sm, pio_program_offset);

    // TODO perform one-time setup of pins here (set for usage with PIO, disable pull up/down, etc) so we don't have to do it every time we change the pin for the program
    // Initialize all capacitive sensors
    // for (uint i = 0; i < NUM_KEYS; i++) {
    //     init_capacitive_sensor(i, FIRST_KEY_PIN + i);
    // }
    
    printf("\nStarting continuous scanning...\n\n");
    
    // Array to store readings
    uint32_t readings[NUM_KEYS];
    
    // Main loop: continuously read and display touch sensor values
    while (true) {
        // Read from all sensor FIFOs
        for (uint i = 0; i < NUM_KEYS; i++) {
            printf("\nReading key %d on pin %d...\n", i, FIRST_KEY_PIN + i);
            capacitive_touch_program_set_pin(pio, sm, pio_program_offset, i + FIRST_KEY_PIN);
            
            // Start the state machine
            pio_sm_set_enabled(pio, sm, true);
            printf("Waiting for data...");

            // Wait for data to be available in FIFO
            while (pio_sm_is_rx_fifo_empty(pio, sm)) {
                tight_loop_contents();
            }
            
            printf("Data available! Reading... ");
            // Read the cycle count from FIFO
            // PIO counts DOWN from 0xFFFFFFFF, so actual count = MAX - reading
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
