#include "driver/ledc.h"

// ==========================================================
// PIN DEFINITIONS (Updated for stability)
// ==========================================================

// --- FIRST BRIDGE ---
const int pin_q1 = 17; // Changed from 13 (LED)
const int pin_q3 = 18; // Changed from 14
const int pin_q2 = 26; 
const int pin_q4 = 27; 

// --- SECOND BRIDGE (Phase Shifted) ---
const int pin_q5 = 25; 
const int pin_q7 = 33; 
const int pin_q6 = 16; // Changed from 12 (Strapping Pin)
const int pin_q8 = 32; 

// ==========================================================
// PWM TIMER SETTINGS
// ==========================================================
const uint32_t FREQ_HZ = 50000;         // 50 kHz
const uint32_t MAX_STEPS = 1024;        // 10-bit resolution
const uint32_t HALF_PERIOD = 512;       // 180 degrees

// ==========================================================
// CALCULATED TIMING
// ==========================================================
const uint32_t SHIFT_TICKS = 124;       // ~43.6 degrees
const uint32_t ACTIVE_DUTY = 307;       // 30% Duty Cycle

// Bridge 1 Reference H-Points
const uint32_t HP_Q1_Q3 = 0;
const uint32_t HP_Q2_Q4 = HALF_PERIOD;

// Bridge 2 Shifted H-Points
const uint32_t HP_Q5_Q7 = SHIFT_TICKS;
const uint32_t HP_Q6_Q8 = (HALF_PERIOD + SHIFT_TICKS) % MAX_STEPS;

// ==========================================================
// HELPER FUNCTION TO CONFIGURE CHANNELS
// ==========================================================
void setup_pwm_channel(int gpio_pin, ledc_channel_t channel, uint32_t hpoint) {
    ledc_channel_config_t conf = {
        .gpio_num   = gpio_pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE, // High speed mode for 50kHz
        .channel    = channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = ACTIVE_DUTY,
        .hpoint     = hpoint
    };
    ledc_channel_config(&conf);
}

void setup() {
    // 1. Configure the shared 10-bit timer
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, 
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // 2. Initialize Bridge 1 (Primary)
    // Leg 1
    setup_pwm_channel(pin_q1, LEDC_CHANNEL_0, HP_Q1_Q3);
    setup_pwm_channel(pin_q3, LEDC_CHANNEL_1, HP_Q1_Q3);
    
    // Leg 2
    setup_pwm_channel(pin_q2, LEDC_CHANNEL_2, HP_Q2_Q4);
    setup_pwm_channel(pin_q4, LEDC_CHANNEL_3, HP_Q2_Q4);

    // 3. Initialize Bridge 2 (Secondary - Phase Shifted)
    // Leg 1
    setup_pwm_channel(pin_q5, LEDC_CHANNEL_4, HP_Q5_Q7);
    setup_pwm_channel(pin_q7, LEDC_CHANNEL_5, HP_Q5_Q7);
    
    // Leg 2
    setup_pwm_channel(pin_q6, LEDC_CHANNEL_6, HP_Q6_Q8);
    setup_pwm_channel(pin_q8, LEDC_CHANNEL_7, HP_Q6_Q8);
}

void loop() {
    // PWM is hardware-controlled. No code needed here.
}