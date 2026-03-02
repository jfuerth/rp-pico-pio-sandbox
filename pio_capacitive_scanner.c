#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "capacitive_touch.pio.h"

// Configuration
#define NUM_KEYS 4
#define FIRST_KEY_PIN 0

// State machines for each key
static uint sm_for_key[NUM_KEYS];
static PIO pio_for_key[NUM_KEYS];

// Initialize a capacitive touch sensor on a given pin
void init_capacitive_sensor(uint key_index, uint pin) {
    // For simplicity, we'll use PIO0 for all sensors
    // Each sensor gets its own state machine
    PIO pio = pio0;
    
    // Try to claim a state machine
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        // If PIO0 is full, try PIO1
        pio = pio1;
        sm = pio_claim_unused_sm(pio, true); // Will panic if none available
    }
    
    // Load the PIO program (only load once per PIO)
    static uint offset_pio0 = 0;
    static uint offset_pio1 = 0;
    static bool loaded_pio0 = false;
    static bool loaded_pio1 = false;
    
    uint offset;
    if (pio == pio0) {
        if (!loaded_pio0) {
            offset = pio_add_program(pio, &capacitive_touch_program);
            offset_pio0 = offset;
            loaded_pio0 = true;
        } else {
            offset = offset_pio0;
        }
    } else {
        if (!loaded_pio1) {
            offset = pio_add_program(pio, &capacitive_touch_program);
            offset_pio1 = offset;
            loaded_pio1 = true;
        } else {
            offset = offset_pio1;
        }
    }
    
    // Initialize the state machine
    capacitive_touch_program_init(pio, sm, offset, pin);
    
    // Store the PIO and SM for later use
    pio_for_key[key_index] = pio;
    sm_for_key[key_index] = sm;
    
    printf("Initialized key %d on GPIO %d using PIO%d SM%d\n", 
           key_index, pin, pio_get_index(pio), sm);
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
    
    // Initialize all capacitive sensors
    for (uint i = 0; i < NUM_KEYS; i++) {
        init_capacitive_sensor(i, FIRST_KEY_PIN + i);
    }
    
    printf("\nStarting continuous scanning...\n\n");
    
    // Array to store readings
    uint32_t readings[NUM_KEYS];
    
    // Main loop: continuously read and display touch sensor values
    while (true) {
        // Read from all sensor FIFOs
        for (uint i = 0; i < NUM_KEYS; i++) {
            PIO pio = pio_for_key[i];
            uint sm = sm_for_key[i];
            
            // Wait for data to be available in FIFO
            while (pio_sm_is_rx_fifo_empty(pio, sm)) {
                tight_loop_contents();
            }
            
            // Read the cycle count from FIFO
            // PIO counts DOWN from 0xFFFFFFFF, so actual count = MAX - reading
            uint32_t countdown_value = pio_sm_get_blocking(pio, sm);
            readings[i] = 0xFFFFFFFF - countdown_value;
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
