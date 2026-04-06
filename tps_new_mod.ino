#include "driver/ledc.h"

// ==========================================================
// PIN DEFINITIONS
// ==========================================================

// --- FIRST BRIDGE ---
const int pin_q1 = 17;
const int pin_q3 = 18;
const int pin_q2 = 26;
const int pin_q4 = 27;

// --- SECOND BRIDGE ---
const int pin_q5 = 25;
const int pin_q7 = 33;
const int pin_q6 = 16;
const int pin_q8 = 32;

// ==========================================================
// PWM TIMER SETTINGS
// ==========================================================
const uint32_t FREQ_HZ     = 50000;  // 50 kHz
const uint32_t MAX_STEPS   = 1024;   // 10-bit resolution
const uint32_t HALF_PERIOD = 512;    // 180 degrees = 512 ticks

// ==========================================================
// TPS PARAMETERS
//
// TPS is the most generalised DAB modulation scheme.
// It introduces THREE independent control variables:
//
//   D1 (INNER_SHIFT_B1): inner phase shift within Bridge 1
//                         (primary). Controls the zero-
//                         voltage dwell width on Vpri.
//
//   D2 (INNER_SHIFT_B2): inner phase shift within Bridge 2
//                         (secondary). Controls the zero-
//                         voltage dwell width on Vsec.
//                         INDEPENDENT of D1 — this is what
//                         separates TPS from EPS and DPS.
//
//   D0 (OUTER_SHIFT):    outer phase shift between Bridge 1
//                         and Bridge 2. Controls power
//                         transfer magnitude and direction.
//
// RELATIONSHIP TO OTHER SCHEMES:
//   SPS : D1=0,  D2=0,  D0=variable   (1 DoF)
//   EPS : D1>0,  D2=0,  D0=variable   (2 DoF, asymmetric)
//   DPS : D1=D2, D2=D1, D0=variable   (2 DoF, symmetric)
//   TPS : D1,    D2,    D0 all free    (3 DoF, fully general)
//
// Power equation (FHA approximation):
//   P = (Vpri * Vsec) / (2*pi*fs*Lk)
//       * sin(pi*D0) * cos(pi*D1) * cos(pi*D2)
//
// The cos terms reduce effective voltage amplitude due to
// the zero-voltage intervals in each bridge waveform.
//
// Constraints (in ticks):
//   0 <= INNER_SHIFT_B1 <= HALF_PERIOD
//   0 <= INNER_SHIFT_B2 <= HALF_PERIOD
//   0 <= OUTER_SHIFT    <= HALF_PERIOD
//   INNER_SHIFT_B1 + OUTER_SHIFT <= HALF_PERIOD
//   INNER_SHIFT_B2 + OUTER_SHIFT <= HALF_PERIOD
//
// To maximise ZVS range and minimise RMS current, D1 and D2
// should be optimised as a function of load and voltage
// conversion ratio (M = n*Vout/Vin). At M=1 (matched
// voltage), D1=D2 gives symmetric optimal operation.
// At M≠1, asymmetric D1≠D2 gives better ZVS coverage.
// ==========================================================
const uint32_t INNER_SHIFT_B1 = 100;  // D1 ~ 35 deg (primary inner)
const uint32_t INNER_SHIFT_B2 = 128;  // D2 ~ 45 deg (secondary inner)
const uint32_t OUTER_SHIFT    = 90;   // D0 ~ 31 deg (inter-bridge)

// ==========================================================
// SAFETY CHECK (evaluated at compile time via static_assert)
// Prevents shoot-through if parameters are misconfigured.
// ==========================================================
static_assert(INNER_SHIFT_B1 + OUTER_SHIFT <= HALF_PERIOD,
    "TPS constraint violated: D1 + D0 must be <= 0.5");
static_assert(INNER_SHIFT_B2 + OUTER_SHIFT <= HALF_PERIOD,
    "TPS constraint violated: D2 + D0 must be <= 0.5");

// ==========================================================
// BRIDGE 1 — PRIMARY (3-level Vpri, inner shift = D1)
//
// Vpri waveform:
//
//  +Vdc  ___                  ___
//       |   |                |   |
//  0 ---     ----        ----     ---...
//                 |      |
//  -Vdc            ------
//
// Zero-voltage dwell width = INNER_SHIFT_B1 ticks
//
// Q1/Q3 turn on at t=0
// Q2/Q4 turn on at t = HALF_PERIOD + INNER_SHIFT_B1
//   (complementary leg, pushed further by inner shift)
// ==========================================================
const uint32_t B1_Q1_Q3_HP = 0;
const uint32_t B1_Q2_Q4_HP = (HALF_PERIOD + INNER_SHIFT_B1) % MAX_STEPS;

// ==========================================================
// BRIDGE 2 — SECONDARY (3-level Vsec, inner shift = D2)
//
// Vsec waveform:
//
//  +Vdc      ___                  ___
//           |   |                |   |
//  0 ---    |    ----        ----
//       ----|                         ---...
//  -Vdc      (shifted by OUTER_SHIFT from Vpri)
//
// Zero-voltage dwell width = INNER_SHIFT_B2 ticks
// Entire Bridge 2 offset by OUTER_SHIFT from Bridge 1.
//
// Q5/Q7 turn on at t = OUTER_SHIFT
// Q6/Q8 turn on at t = HALF_PERIOD + INNER_SHIFT_B2
//                      + OUTER_SHIFT
// ==========================================================
const uint32_t B2_Q5_Q7_HP = OUTER_SHIFT % MAX_STEPS;
const uint32_t B2_Q6_Q8_HP = (HALF_PERIOD + INNER_SHIFT_B2 + OUTER_SHIFT)
                               % MAX_STEPS;

// ==========================================================
// DUTY CYCLES
//
// Each bridge has its own active duty, reduced by its own
// inner shift to prevent shoot-through.
//
// Safe maximum per bridge:
//   duty_B1 = HALF_PERIOD - INNER_SHIFT_B1 - DEADTIME
//   duty_B2 = HALF_PERIOD - INNER_SHIFT_B2 - DEADTIME
//
// Since D1 ≠ D2 in general TPS, the two bridges will have
// DIFFERENT duty cycles — this is the key hardware
// distinction vs SPS/DPS where duties are equal.
// ==========================================================
const uint32_t DEADTIME       = 10;
const uint32_t ACTIVE_DUTY_B1 = HALF_PERIOD - INNER_SHIFT_B1 - DEADTIME;
const uint32_t ACTIVE_DUTY_B2 = HALF_PERIOD - INNER_SHIFT_B2 - DEADTIME;

// ==========================================================
// HELPER FUNCTION
// ==========================================================
void setup_pwm_channel(int gpio_pin,
                       ledc_channel_t channel,
                       uint32_t hpoint,
                       uint32_t duty) {
    ledc_channel_config_t conf = {
        .gpio_num   = gpio_pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = duty,
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
    // 2. BRIDGE 1 (Primary) — 3-level Vpri with D1 inner shift
    //
    // CH0: Q1 — reference, turns on at t=0
    // CH1: Q3 — same leg as Q1
    // CH2: Q2 — complementary leg: HALF_PERIOD + INNER_SHIFT_B1
    // CH3: Q4 — same leg as Q2
    //
    // Active duty reduced by INNER_SHIFT_B1 to match the
    // shortened ON-time of each pulse in the 3-level wave.
    // ----------------------------------------------------------
    setup_pwm_channel(pin_q1, LEDC_CHANNEL_0, B1_Q1_Q3_HP, ACTIVE_DUTY_B1);
    setup_pwm_channel(pin_q3, LEDC_CHANNEL_1, B1_Q1_Q3_HP, ACTIVE_DUTY_B1);

    setup_pwm_channel(pin_q2, LEDC_CHANNEL_2, B1_Q2_Q4_HP, ACTIVE_DUTY_B1);
    setup_pwm_channel(pin_q4, LEDC_CHANNEL_3, B1_Q2_Q4_HP, ACTIVE_DUTY_B1);

    // ----------------------------------------------------------
    // 3. BRIDGE 2 (Secondary) — 3-level Vsec with D2 inner shift
    //
    // CH4: Q5 — outer-shifted by OUTER_SHIFT from Bridge 1
    // CH5: Q7 — same leg as Q5
    // CH6: Q6 — HALF_PERIOD + INNER_SHIFT_B2 + OUTER_SHIFT
    // CH7: Q8 — same leg as Q6
    //
    // Active duty reduced by INNER_SHIFT_B2 independently.
    // This independent duty is what makes TPS fully general.
    // ----------------------------------------------------------
    setup_pwm_channel(pin_q5, LEDC_CHANNEL_4, B2_Q5_Q7_HP, ACTIVE_DUTY_B2);
    setup_pwm_channel(pin_q7, LEDC_CHANNEL_5, B2_Q5_Q7_HP, ACTIVE_DUTY_B2);

    setup_pwm_channel(pin_q6, LEDC_CHANNEL_6, B2_Q6_Q8_HP, ACTIVE_DUTY_B2);
    setup_pwm_channel(pin_q8, LEDC_CHANNEL_7, B2_Q6_Q8_HP, ACTIVE_DUTY_B2);

    // ----------------------------------------------------------
    // Debug output
    // ----------------------------------------------------------
    Serial.println("TPS Modulation Active");
    Serial.println("-------------------------------");
    Serial.print("D1 - Inner Shift B1 ticks : "); Serial.println(INNER_SHIFT_B1);
    Serial.print("D2 - Inner Shift B2 ticks : "); Serial.println(INNER_SHIFT_B2);
    Serial.print("D0 - Outer Shift    ticks : "); Serial.println(OUTER_SHIFT);
    Serial.print("Bridge 1 Active Duty ticks: "); Serial.println(ACTIVE_DUTY_B1);
    Serial.print("Bridge 2 Active Duty ticks: "); Serial.println(ACTIVE_DUTY_B2);
    Serial.println("-------------------------------");
    Serial.println("Both bridges run 3-level waveforms.");
    Serial.println("D1 and D2 are fully independent.");
}

// ==========================================================
// LOOP
//
// In closed-loop TPS, all three variables are updated
// dynamically by the PI/MPC controller every cycle.
//
// Typical closed-loop strategy:
//   - D0 (OUTER_SHIFT) tracks the power demand from the
//     outer voltage PI loop
//   - D1 and D2 are scheduled from a lookup table indexed
//     by load current and voltage ratio M = n*Vout/Vin
//     to maintain ZVS and minimise RMS current
//
// To update all three variables dynamically:
//
//   uint32_t new_d0 = outer_pi_output;
//   uint32_t new_d1 = lut_d1[load_index][m_index];
//   uint32_t new_d2 = lut_d2[load_index][m_index];
//
//   uint32_t new_duty_b1 = HALF_PERIOD - new_d1 - DEADTIME;
//   uint32_t new_duty_b2 = HALF_PERIOD - new_d2 - DEADTIME;
//
//   // Update Bridge 1 hpoints + duty
//   ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
//       LEDC_CHANNEL_0, new_duty_b1, 0);
//   ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
//       LEDC_CHANNEL_2, new_duty_b1,
//       (HALF_PERIOD + new_d1) % MAX_STEPS);
//
//   // Update Bridge 2 hpoints + duty
//   ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
//       LEDC_CHANNEL_4, new_duty_b2,
//       new_d0 % MAX_STEPS);
//   ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
//       LEDC_CHANNEL_6, new_duty_b2,
//       (HALF_PERIOD + new_d2 + new_d0) % MAX_STEPS);
//
//   // Repeat for CH1, CH3, CH5, CH7 (paired channels)
//
// NOTE: On ESP32, ledc_set_duty_and_update() is the safe
// way to change both duty and hpoint atomically. Changing
// them separately risks a glitch cycle on the output.
// ==========================================================
void loop() {
    // Static open-loop: PWM is hardware-controlled.
    // Swap in closed-loop update code above when integrating
    // with your PI controller and ADC voltage feedback.
}