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
// EPS PARAMETERS
//
// EPS introduces TWO control variables:
//
//   INNER_SHIFT (δin): inner phase shift applied ONLY within
//                      Bridge 1 (primary). This creates a
//                      3-level waveform on Vpri.
//                      Controls the zero-voltage interval
//                      on the primary side only.
//
//   OUTER_SHIFT (δout): outer phase shift between Bridge 1
//                       and Bridge 2, same as SPS.
//                       Controls power transfer magnitude
//                       and direction.
//
// KEY DIFFERENCE FROM DPS:
//   DPS  — inner shift applied symmetrically to BOTH bridges
//   EPS  — inner shift applied to PRIMARY bridge ONLY
//          Secondary (Bridge 2) runs as a plain 2-level
//          square wave, exactly like SPS
//
// Power equation (from literature):
//   P = (n*Vsec*Vpri / 4*Lk*fs) *
//       [δin*(1 - δin - 2*δout) + 2*δout*(1 - δout)]
//
// Constraints:
//   0 <= δin  <= 1  (normalised), i.e. 0 to HALF_PERIOD ticks
//   0 <= δout <= 1  (normalised), i.e. 0 to HALF_PERIOD ticks
//   δin + 2*δout <= 1
//   In ticks: INNER_SHIFT + 2*OUTER_SHIFT <= HALF_PERIOD
//
// Example values below give moderate power transfer.
// Increase OUTER_SHIFT to transfer more power.
// Increase INNER_SHIFT to widen the zero interval on Vpri
// and reduce circulating current at light load.
// ==========================================================
const uint32_t INNER_SHIFT = 128;   // δin  ~ 45 deg (primary inner shift)
const uint32_t OUTER_SHIFT = 100;   // δout ~ 35 deg (inter-bridge shift)

// ==========================================================
// BRIDGE 1 — PRIMARY (3-level Vpri waveform)
//
// Vpri waveform shape:
//
//  +Vdc  ___              ___
//       |   |            |   |
//  0 ---     ----    ----     ---...
//                |  |
//  -Vdc          ---
//
// Q1 & Q3 turn ON at t=0         (hpoint = 0)
// Q2 & Q4 turn ON at t=INNER_SHIFT later
//   → this creates the zero-voltage dwell on Vpri
//
// Both legs still run at 50% duty relative to their own
// hpoint, so the effective pulse width is:
//   ACTIVE_DUTY = HALF_PERIOD - INNER_SHIFT - DEADTIME
// ==========================================================
const uint32_t B1_Q1_Q3_HP    = 0;
const uint32_t B1_Q2_Q4_HP    = (HALF_PERIOD + INNER_SHIFT) % MAX_STEPS;
// Q2/Q4 are the complementary leg, offset by HALF_PERIOD
// then additionally pushed by INNER_SHIFT to create the
// zero-voltage dwell at the centre of the waveform.

// ==========================================================
// BRIDGE 2 — SECONDARY (plain 2-level Vsec, like SPS)
//
// No inner shift here — Bridge 2 just runs with a fixed
// 50% square wave, shifted by OUTER_SHIFT from Bridge 1.
// This is what distinguishes EPS from DPS.
//
// Vsec waveform shape (standard square wave):
//
//  +Vdc  ________
//       |        |
//  0 ---          --------...
//  -Vdc            ________
//
// ==========================================================
const uint32_t B2_Q5_Q7_HP = OUTER_SHIFT % MAX_STEPS;
const uint32_t B2_Q6_Q8_HP = (HALF_PERIOD + OUTER_SHIFT) % MAX_STEPS;

// ==========================================================
// DUTY CYCLES
//
// Bridge 1: reduced duty to accommodate the inner shift.
//   If duty + hpoint overlap with the complementary leg,
//   shoot-through occurs. The safe maximum is:
//   ACTIVE_DUTY_B1 = HALF_PERIOD - INNER_SHIFT - DEADTIME
//
// Bridge 2: standard 50% duty (no inner shift).
//   ACTIVE_DUTY_B2 = HALF_PERIOD - DEADTIME
//
// DEADTIME: small guard margin in ticks to prevent
//   simultaneous conduction. 10 ticks ~ 1% at 50kHz.
// ==========================================================
const uint32_t DEADTIME        = 10;
const uint32_t ACTIVE_DUTY_B1  = HALF_PERIOD - INNER_SHIFT - DEADTIME;
const uint32_t ACTIVE_DUTY_B2  = HALF_PERIOD - DEADTIME;

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
    // 2. BRIDGE 1 (Primary) — 3-level Vpri
    //
    // CH0: Q1 — reference, turns on at t=0
    // CH1: Q3 — same leg as Q1, same hpoint
    // CH2: Q2 — complementary leg, offset by HALF_PERIOD
    //           + INNER_SHIFT to create zero-voltage dwell
    // CH3: Q4 — same leg as Q2, same hpoint
    //
    // Result: Vpri has a zero-voltage interval of width
    //         INNER_SHIFT ticks at each transition.
    // ----------------------------------------------------------
    setup_pwm_channel(pin_q1, LEDC_CHANNEL_0, B1_Q1_Q3_HP, ACTIVE_DUTY_B1);
    setup_pwm_channel(pin_q3, LEDC_CHANNEL_1, B1_Q1_Q3_HP, ACTIVE_DUTY_B1);

    setup_pwm_channel(pin_q2, LEDC_CHANNEL_2, B1_Q2_Q4_HP, ACTIVE_DUTY_B1);
    setup_pwm_channel(pin_q4, LEDC_CHANNEL_3, B1_Q2_Q4_HP, ACTIVE_DUTY_B1);

    // ----------------------------------------------------------
    // 3. BRIDGE 2 (Secondary) — plain 2-level Vsec (SPS-style)
    //
    // CH4: Q5 — shifted by OUTER_SHIFT from Bridge 1
    // CH5: Q7 — same leg as Q5
    // CH6: Q6 — complementary leg, HALF_PERIOD + OUTER_SHIFT
    // CH7: Q8 — same leg as Q6
    //
    // Result: Vsec is a standard 50% square wave, phase-shifted
    //         from Vpri by OUTER_SHIFT ticks. No inner shift.
    // ----------------------------------------------------------
    setup_pwm_channel(pin_q5, LEDC_CHANNEL_4, B2_Q5_Q7_HP, ACTIVE_DUTY_B2);
    setup_pwm_channel(pin_q7, LEDC_CHANNEL_5, B2_Q5_Q7_HP, ACTIVE_DUTY_B2);

    setup_pwm_channel(pin_q6, LEDC_CHANNEL_6, B2_Q6_Q8_HP, ACTIVE_DUTY_B2);
    setup_pwm_channel(pin_q8, LEDC_CHANNEL_7, B2_Q6_Q8_HP, ACTIVE_DUTY_B2);

    // ----------------------------------------------------------
    // Debug output
    // ----------------------------------------------------------
    Serial.println("EPS Modulation Active");
    Serial.println("-------------------------------");
    Serial.print("Inner Shift (δin)  ticks: "); Serial.println(INNER_SHIFT);
    Serial.print("Outer Shift (δout) ticks: "); Serial.println(OUTER_SHIFT);
    Serial.print("Bridge 1 Duty ticks:      "); Serial.println(ACTIVE_DUTY_B1);
    Serial.print("Bridge 2 Duty ticks:      "); Serial.println(ACTIVE_DUTY_B2);
    Serial.println("-------------------------------");
    Serial.println("Bridge 2 runs as plain 2-level (SPS-style).");
    Serial.println("Inner shift applied to Bridge 1 only.");
}

// ==========================================================
// LOOP
//
// Static open-loop operation. For closed-loop, the PI
// controller output adjusts OUTER_SHIFT dynamically.
// INNER_SHIFT can also be scheduled as a function of
// load to minimise circulating current at light load.
//
// To update dynamically:
//
//   uint32_t new_outer = pi_output;
//
//   // Update Bridge 2 hpoints (outer shift)
//   ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
//       LEDC_CHANNEL_4, ACTIVE_DUTY_B2,
//       new_outer % MAX_STEPS);
//   ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
//       LEDC_CHANNEL_6, ACTIVE_DUTY_B2,
//       (HALF_PERIOD + new_outer) % MAX_STEPS);
//   // Repeat for CH5, CH7
// ==========================================================
void loop() {
    // PWM is hardware-controlled in open-loop mode.
}