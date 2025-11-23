// SPDX-License-Identifier: MIT
// DMA-accelerated PID motor control for RP2040/RP2350

#ifndef _PBDRV_PID_DMA_RP2040_H_
#define _PBDRV_PID_DMA_RP2040_H_

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"
#include "hardware/dma.h"

#ifndef PID_DMA_MAX_MOTORS
#define PID_DMA_MAX_MOTORS 4
#endif

#ifndef PID_DMA_UPDATE_RATE_HZ
#define PID_DMA_UPDATE_RATE_HZ 1000
#endif

#define PID_FP_SHIFT 16
#define PID_FP_ONE (1 << PID_FP_SHIFT)
#define PID_DMA_BUFFER_SIZE 16

typedef struct __attribute__((aligned(32))) {
    volatile int32_t target_position;
    volatile int32_t target_speed;
    volatile int32_t current_position;
    volatile int32_t last_position;
    volatile uint32_t timestamp_us;
    volatile uint32_t last_timestamp_us;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    volatile int32_t integral;
    volatile int32_t last_error;
    int32_t integral_limit;
    volatile int32_t pwm_output;
    volatile int32_t computed_speed;
    volatile uint8_t enabled;
    volatile uint8_t mode;
    volatile uint8_t stalled;
    uint8_t _pad[1];
} pid_dma_state_t;

typedef struct {
    PIO encoder_pio;
    uint encoder_sm;
    uint encoder_pin_a;
    uint encoder_pin_b;
    int encoder_dma_chan;
    PIO pwm_pio;
    uint pwm_sm;
    uint pwm_pin_a;
    uint pwm_pin_b;
    int pwm_dma_chan;
    int timer_dma_chan;
    int ctrl_dma_chan;
    uint32_t counts_per_rev;
} pid_dma_hw_config_t;

typedef struct {
    pid_dma_state_t state;
    pid_dma_hw_config_t hw;
    uint32_t dma_ctrl_block[4] __attribute__((aligned(16)));
    uint32_t encoder_buffer[PID_DMA_BUFFER_SIZE];
    volatile uint32_t encoder_write_idx;
    bool initialized;
} pid_dma_motor_t;

void pid_dma_init(void);
int pid_dma_motor_setup(uint8_t motor_id, const pid_dma_hw_config_t *config);
void pid_dma_set_gains(uint8_t motor_id, int32_t kp, int32_t ki, int32_t kd);
void pid_dma_set_position_target(uint8_t motor_id, int32_t position);
void pid_dma_set_speed_target(uint8_t motor_id, int32_t speed);
void pid_dma_set_pwm(uint8_t motor_id, int32_t pwm);
void pid_dma_enable(uint8_t motor_id, bool enable);
int pid_dma_get_state(uint8_t motor_id, int32_t *position, int32_t *speed, int32_t *pwm);
bool pid_dma_is_stalled(uint8_t motor_id);
void pid_dma_reset_position(uint8_t motor_id);
void pid_dma_deinit(void);
volatile pid_dma_state_t* pid_dma_get_state_ptr(uint8_t motor_id);
void pid_dma_force_update(uint8_t motor_id);

#endif
