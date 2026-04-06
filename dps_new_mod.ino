#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// ==========================================================
// PIN DEFINITIONS
// ==========================================================

// --- FIRST BRIDGE ---
const int pin_q1 = 17;
const int pin_q3 = 18;
const int pin_q2 = 26;
const int pin_q4 = 27;

// --- SECOND BRIDGE (Phase Shifted) ---
const int pin_q5 = 25;
const int pin_q7 = 33;
const int pin_q6 = 16;
const int pin_q8 = 32;

// ==========================================================
// PWM TIMER SETTINGS
// ==========================================================
const uint32_t FREQ_HZ    = 50000;   // 50 kHz
const uint32_t MAX_STEPS  = 1024;    // 10-bit resolution
const uint32_t HALF_PERIOD = 512;    // 180 degrees = 512 ticks

// ==========================================================
// DPS PARAMETERS
// Tune these two values to control power flow:
//   INNER_SHIFT (D1): inner phase shift within each bridge
//                     controls the zero-voltage interval
//                     (quasi-square wave width)
//   OUTER_SHIFT (D0): outer phase shift between Bridge 1
//                     and Bridge 2
//                     controls power transfer direction
//
// Constraints:
//   0 <= INNER_SHIFT <= HALF_PERIOD
//   0 <= OUTER_SHIFT <= HALF_PERIOD
//   INNER_SHIFT + OUTER_SHIFT <= HALF_PERIOD
//
// Power equation:
//   P = (Vpri * Vsec) / (2*pi*fs*Lk) * sin(pi*D1) * cos(pi*D0)
//
// Example values below give moderate power transfer
// with reduced circulating current vs SPS.
// ==========================================================
const uint32_t INNER_SHIFT = 128;  // D1 ~ 45 deg  (inner, within bridge)
const uint32_t OUTER_SHIFT = 100;  // D0 ~ 35 deg  (outer, between bridges)

// ==========================================================
// BRIDGE 1 - PRIMARY
// Q1/Q3 and Q2/Q4 are separated by INNER_SHIFT
// creating a 3-level quasi-square waveform on Vpri
//
//  Vpri waveform:
//  +Vdc  ____          ____
//       |    |        |    |
//  0 ---      --------      ---...
//              ____
//  -Vdc       |    |
//
// The zero-voltage interval width = INNER_SHIFT ticks
// ==========================================================
const uint32_t B1_Q1_Q3_HP = 0;
const uint32_t B1_Q2_Q4_HP = HALF_PERIOD;

// Q1 and Q3 share the same hpoint (same leg of Bridge 1)
// Q2 and Q4 are offset by INNER_SHIFT from Q1/Q3
// This creates the inner phase shift within Bridge 1
const uint32_t B1_INNER_Q2_HP = (B1_Q1_Q3_HP + INNER_SHIFT) % MAX_STEPS;
const uint32_t B1_INNER_Q4_HP = (B1_Q2_Q4_HP + INNER_SHIFT) % MAX_STEPS;

// ==========================================================
// BRIDGE 2 - SECONDARY
// Entire Bridge 2 is shifted by OUTER_SHIFT relative to
// Bridge 1, then inner shift is applied within Bridge 2
// as well (symmetric DPS).
//
// Vsec waveform is also 3-level, shifted by OUTER_SHIFT
// ==========================================================
const uint32_t B2_Q5_Q7_HP = (B1_Q1_Q3_HP + OUTER_SHIFT) % MAX_STEPS;
const uint32_t B2_Q6_Q8_HP = (B1_Q2_Q4_HP + OUTER_SHIFT) % MAX_STEPS;

const uint32_t B2_INNER_Q5_HP = (B2_Q5_Q7_HP + INNER_SHIFT) % MAX_STEPS;
const uint32_t B2_INNER_Q8_HP = (B2_Q6_Q8_HP + INNER_SHIFT) % MAX_STEPS;

// ==========================================================
// DUTY CYCLE
// In DPS, duty cycle represents the ON-time of each pulse
// within the quasi-square wave. Keep below HALF_PERIOD
// minus INNER_SHIFT to avoid shoot-through.
// ==========================================================
const uint32_t ACTIVE_DUTY = (HALF_PERIOD - INNER_SHIFT - 10);
// The -10 is a dead-time margin. Adjust as needed.

// ==========================================================
// HELPER FUNCTION
// ==========================================================
void setup_pwm_channel(int gpio_pin, ledc_channel_t channel, uint32_t hpoint) {
    ledc_channel_config_t conf = {
        .gpio_num   = gpio_pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = ACTIVE_DUTY,
        .hpoint     = hpoint
    };
    ledc_channel_config(&conf);
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
    Serial.begin(115200);

    // 1. Shared 10-bit timer
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // ----------------------------------------------------------
    // 2. BRIDGE 1 (Primary) — 3-level quasi-square Vpri
    //
    // CH0: Q1 — reference leg 1 start
    // CH1: Q3 — same as Q1 (diagonal pair)
    // CH2: Q2 — inner-shifted leg 2 start
    // CH3: Q4 — same as Q2 (diagonal pair)
    // ----------------------------------------------------------
    setup_pwm_channel(pin_q1, LEDC_CHANNEL_0, B1_Q1_Q3_HP);
    setup_pwm_channel(pin_q3, LEDC_CHANNEL_1, B1_Q1_Q3_HP);

    setup_pwm_channel(pin_q2, LEDC_CHANNEL_2, B1_INNER_Q2_HP);
    setup_pwm_channel(pin_q4, LEDC_CHANNEL_3, B1_INNER_Q4_HP);

    // ----------------------------------------------------------
    // 3. BRIDGE 2 (Secondary) — 3-level quasi-square Vsec
    //    offset by OUTER_SHIFT from Bridge 1
    //
    // CH4: Q5 — outer-shifted leg 1 start
    // CH5: Q7 — same as Q5
    // CH6: Q6 — outer + inner shifted leg 2 start
    // CH7: Q8 — same as Q6
    // ----------------------------------------------------------
    setup_pwm_channel(pin_q5, LEDC_CHANNEL_4, B2_Q5_Q7_HP);
    setup_pwm_channel(pin_q7, LEDC_CHANNEL_5, B2_Q5_Q7_HP);

    setup_pwm_channel(pin_q6, LEDC_CHANNEL_6, B2_INNER_Q5_HP);
    setup_pwm_channel(pin_q8, LEDC_CHANNEL_7, B2_INNER_Q8_HP);

    Serial.println("DPS Modulation Active");
    Serial.print("Inner Shift (D1) ticks: "); Serial.println(INNER_SHIFT);
    Serial.print("Outer Shift (D0) ticks: "); Serial.println(OUTER_SHIFT);
    Serial.print("Active Duty ticks:       "); Serial.println(ACTIVE_DUTY);
}

// ==========================================================
// LOOP
// Optionally sweep OUTER_SHIFT via Serial for testing.
// In closed-loop hardware, the PI controller would call
// ledc_set_duty() + ledc_update_duty() here to
// dynamically update OUTER_SHIFT based on Vout error.
// ==========================================================
void loop() {
    // Static open-loop: PWM is hardware-controlled.
    // For closed-loop, compute new hpoints from PI output
    // and update like this:
    //
    // uint32_t new_outer = pi_output;  // from your PI controller
    // ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4,
    //                          ACTIVE_DUTY, new_outer % MAX_STEPS);
    // ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5,
    //                          ACTIVE_DUTY, new_outer % MAX_STEPS);
    // etc.
}