// SPDX-License-Identifier: MIT
// RP2350B 12-encoder driver with 3 PIO blocks

#include <pbdrv/config.h>

#if PBDRV_CONFIG_COUNTER_RP2350_PIO

#include <stdint.h>
#include <stdio.h>

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#include "quadrature_encoder.pio.h"
#include <pbio/error.h>

#define NUM_ENCODERS 12

typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    PIO pio;
    uint sm;
    int dma_chan;
    volatile int32_t count;
    uint8_t last_state;
    volatile int32_t velocity;  // Ticks per second
    int32_t last_count;
    uint32_t last_time_us;
} encoder_t;

static encoder_t encoders[NUM_ENCODERS];

// State transition table
static const int8_t state_table[4][4] = {
    { 0, -1,  1,  0},
    { 1,  0,  0, -1},
    {-1,  0,  0,  1},
    { 0,  1, -1,  0}
};

void pbdrv_counter_init(void) {
    printf("\n=== Initializing 12 PIO Encoders ===\n");
    printf("Target: RP2350B (3 PIO blocks)\n");
    
    // Load PIO programs
    uint offset0 = pio_add_program(pio0, &quadrature_encoder_program);
    uint offset1 = pio_add_program(pio1, &quadrature_encoder_program);
    uint offset2 = pio_add_program(pio2, &quadrature_encoder_program);
    
    // Pin assignments for 12 encoders
    const uint8_t encoder_pins[NUM_ENCODERS][2] = {
        {24, 25}, {26, 27}, {28, 29}, {30, 31},  // PIO0 (encoders 0-3)
        {32, 33}, {34, 35}, {36, 37}, {38, 39},  // PIO1 (encoders 4-7)
        {40, 41}, {42, 43}, {44, 45}, {46, 47}   // PIO2 (encoders 8-11)
    };
    
    for (int i = 0; i < NUM_ENCODERS; i++) {
        encoder_t *enc = &encoders[i];
        
        enc->pin_a = encoder_pins[i][0];
        enc->pin_b = encoder_pins[i][1];
        
        // Assign to PIO block (4 encoders per PIO)
        if (i < 4) {
            enc->pio = pio0;
            enc->sm = i;
            quadrature_encoder_program_init(enc->pio, enc->sm, offset0, enc->pin_a);
        } else if (i < 8) {
            enc->pio = pio1;
            enc->sm = i - 4;
            quadrature_encoder_program_init(enc->pio, enc->sm, offset1, enc->pin_a);
        } else {
            enc->pio = pio2;
            enc->sm = i - 8;
            quadrature_encoder_program_init(enc->pio, enc->sm, offset2, enc->pin_a);
        }
        
        enc->count = 0;
        enc->velocity = 0;
        enc->last_count = 0;
        enc->last_time_us = 0;
        
        // Read initial state
        uint8_t a = gpio_get(enc->pin_a);
        uint8_t b = gpio_get(enc->pin_b);
        enc->last_state = (a << 1) | b;
        
        // Setup DMA
        enc->dma_chan = dma_claim_unused_channel(true);
        
        dma_channel_config cfg = dma_channel_get_default_config(enc->dma_chan);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, false);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_dreq(&cfg, pio_get_dreq(enc->pio, enc->sm, false));
        
        printf("  Encoder %2d: GP%d/GP%d, PIO%d.SM%d, DMA ch%d\n",
               i, enc->pin_a, enc->pin_b, 
               (enc->pio == pio0) ? 0 : (enc->pio == pio1) ? 1 : 2,
               enc->sm, enc->dma_chan);
    }
    
    printf("12 encoders initialized\n");
}

// Update encoders (called at 1 kHz)
void pbdrv_counter_update(void) {
    uint32_t now = time_us_32();
    
    for (int i = 0; i < NUM_ENCODERS; i++) {
        encoder_t *enc = &encoders[i];
        
        // Read new states from PIO
        while (!pio_sm_is_rx_fifo_empty(enc->pio, enc->sm)) {
            uint32_t new_state = pio_sm_get(enc->pio, enc->sm);
            new_state &= 0x03;
            
            int8_t delta = state_table[enc->last_state][new_state];
            enc->count += delta;
            enc->last_state = new_state;
        }
        
        // Calculate velocity every 10ms
        if (now - enc->last_time_us >= 10000) {
            int32_t delta_count = enc->count - enc->last_count;
            uint32_t delta_time = now - enc->last_time_us;
            
            // Velocity in ticks/second
            enc->velocity = (delta_count * 1000000) / delta_time;
            
            enc->last_count = enc->count;
            enc->last_time_us = now;
        }
    }
}

// Get position
int32_t pbdrv_counter_get_count_simple(uint8_t id) {
    if (id >= NUM_ENCODERS) return 0;
    return encoders[id].count;
}

// Get velocity
int32_t pbdrv_counter_get_rate_simple(uint8_t id) {
    if (id >= NUM_ENCODERS) return 0;
    return encoders[id].velocity;
}

// Reset position
void pbdrv_counter_reset(uint8_t id) {
    if (id < NUM_ENCODERS) {
        encoders[id].count = 0;
        encoders[id].last_count = 0;
    }
}

// Get all encoder counts (for debugging)
void pbdrv_counter_get_all_counts(int32_t *counts) {
    for (int i = 0; i < NUM_ENCODERS; i++) {
        counts[i] = encoders[i].count;
    }
}

#endif // PBDRV_CONFIG_COUNTER_RP2350_PIO
