# DABconvertor

## Closed Loop Control of a Dual Active Bridge Converter
**B.Tech Final Year Project | VIT Chennai | April 2026**
**Team:** Arun R (22BEE1052) · Utsav Johri (22BEE1092) · P Harini (22BEE1154)
**Guide:** Dr. Jyotismita Mishra, School of Electrical Engineering

---

## Overview
This repository contains the ESP32 firmware and MATLAB/Simulink models for a
closed-loop controlled Dual Active Bridge (DAB) DC-DC converter. The system
implements four phase-shift modulation strategies — SPS, EPS, DPS, and TPS —
with a cascaded PI control loop for stable output voltage regulation.

---

## Specs
| Parameter | Value |
|---|---|
| Input Voltage | 100V (24V for hardware testing) |
| Output Voltage | 48V |
| Switching Frequency | 50 kHz |
| Rated Power | 500W |
| Transformer Turns Ratio | 2.083 : 1 |
| Leakage Inductance | 187.5 µH |
| Output Capacitance | 100 µF |
| Load Resistance | 20.1 Ω |

---

## Hardware
- **Microcontroller:** ESP32 (MCPWM module for PWM generation)
- **Switches:** FCP20N60 MOSFETs (600V, 20A)
- **Gate Driver:** TLP152 optocouplers (isolated ±15V drive)
- **Transformer:** 100V/48V 500W ferrite core HFT

---

## Modulation Strategies
| Strategy | DoF | ZVS Range | RMS Current | Complexity |
|---|---|---|---|---|
| SPS | 1 | Low | High | Simple |
| EPS | 2 | Medium | Medium | Moderate |
| DPS | 2 | Medium | Low | Moderate |
| TPS | 3 | High | Lowest | Complex |

---

## Control Architecture
Cascaded dual PI loop:
- **Outer loop** — output voltage regulation
- **Inner loop** — inductor current control
- Phase shift command dynamically fed to ESP32 MCPWM

---

## Repository Structure
