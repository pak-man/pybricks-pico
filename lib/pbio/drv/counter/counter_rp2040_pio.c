// SPDX-License-Identifier: MIT
// RP2040 PIO + DMA quadrature encoder driver

#include <pbdrv/config.h>

#if PBDRV_CONFIG_COUNTER_RP2040_PIO

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "quadrature_encoder.pio.h"

#include <pbio/error.h>

// Encoder data structure
typedef struct {
    uint8_t pin_a;          // Encoder A pin
    uint8_t pin_b;          // Encoder B pin
    PIO pio;                // PIO instance
    uint sm;                // State machine number
    int dma_chan;           // DMA channel for reading
    volatile int32_t count; // Current position
    uint8_t last_state;     // Previous state for decoding
} encoder_t;

static encoder_t encoders[PBDRV_CONFIG_COUNTER_NUM_DEV];

// State transition lookup table for direction decoding
// [old_state][new_state] = direction (+1, -1, or 0)
static const int8_t state_table[4][4] = {
    { 0, -1,  1,  0},  // From 00
    { 1,  0,  0, -1},  // From 01
    {-1,  0,  0,  1},  // From 10
    { 0,  1, -1,  0}   // From 11
};

// DMA buffer for reading PIO FIFOs
static volatile uint32_t dma_buffer[PBDRV_CONFIG_COUNTER_NUM_DEV];

void pbdrv_counter_init(void) {
    printf("Initializing PIO encoders with DMA...\n");
    
    // Load PIO program once
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);
    
    // Configure each encoder
    for (int i = 0; i < PBDRV_CONFIG_COUNTER_NUM_DEV; i++) {
        encoder_t *enc = &encoders[i];
        
        // Assign pins: Port A=2-3, B=6-7, C=10-11, D=14-15
        enc->pin_a = 2 + (i * 4);
        enc->pin_b = enc->pin_a + 1;
        enc->pio = pio0;
        enc->sm = i;  // Use state machines 0-3
        enc->count = 0;
        
        // Initialize PIO state machine
        quadrature_encoder_program_init(enc->pio, enc->sm, offset, enc->pin_a);
        
        // Read initial state
        uint8_t a = gpio_get(enc->pin_a);
        uint8_t b = gpio_get(enc->pin_b);
        enc->last_state = (a << 1) | b;
        
        // Setup DMA channel for this encoder
        enc->dma_chan = dma_claim_unused_channel(true);
        
        dma_channel_config cfg = dma_channel_get_default_config(enc->dma_chan);
        channel_config_set_read_increment(&cfg, false);  // Always read from FIFO
        channel_config_set_write_increment(&cfg, false); // Always write to same buffer
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_dreq(&cfg, pio_get_dreq(enc->pio, enc->sm, false)); // RX FIFO
        
        // Configure DMA
        dma_channel_configure(
            enc->dma_chan,
            &cfg,
            (void*)&dma_buffer[i],           // Write to buffer
            &enc->pio->rxf[enc->sm],         // Read from PIO FIFO
            0xFFFFFFFF,                      // Transfer forever (will be restarted)
            true                             // Start now
        );
        
        printf("  Encoder %d: GPIO %d-%d, PIO0.SM%d, DMA ch%d\n", 
               i, enc->pin_a, enc->pin_b, enc->sm, enc->dma_chan);
    }
    
    printf("PIO encoders initialized with DMA\n");
}

// Process encoder state changes (called from timer)
void pbdrv_counter_update(void) {
    for (int i = 0; i < PBDRV_CONFIG_COUNTER_NUM_DEV; i++) {
        encoder_t *enc = &encoders[i];
        
        // Check if PIO has new data
        if (!pio_sm_is_rx_fifo_empty(enc->pio, enc->sm)) {
            // Read state from PIO (via DMA buffer or direct)
            uint32_t new_state = pio_sm_get_blocking(enc->pio, enc->sm);
            new_state &= 0x03;  // Mask to 2 bits
            
            // Decode direction
            int8_t delta = state_table[enc->last_state][new_state];
            enc->count += delta;
            enc->last_state = new_state;
        }
    }
}

// Get encoder count
int32_t pbdrv_counter_get_count_simple(uint8_t id) {
    if (id >= PBDRV_CONFIG_COUNTER_NUM_DEV) {
        return 0;
    }
    return encoders[id].count;
}

// Reset encoder count
void pbdrv_counter_reset(uint8_t id) {
    if (id < PBDRV_CONFIG_COUNTER_NUM_DEV) {
        encoders[id].count = 0;
    }
}

// Get encoder rate (velocity)
int32_t pbdrv_counter_get_rate_simple(uint8_t id) {
    // TODO: Calculate velocity from position changes
    // For now, return 0
    return 0;
}

#endif // PBDRV_CONFIG_COUNTER_RP2040_PIO
