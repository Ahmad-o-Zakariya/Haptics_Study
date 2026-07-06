# Haptics_Study

A collection of embedded haptic rendering experiments developed on the **ATmega328P (Arduino Uno)** platform. This repository documents the evolution of a **1-DOF force-feedback haptic interface**, from a production-grade virtual wall renderer to advanced research implementations including **fractional-order viscoelastic models**, **dual-rate control**, **direct port manipulation**, and multiple motor driver configurations.

The project began as a high-performance virtual wall implementation and has since evolved into a research platform for studying real-time embedded haptic rendering, control system stability, and computational optimization on resource-constrained microcontrollers.

Following successful validation and technical review by senior faculty, the original virtual wall implementation was officially approved as the benchmark instructional demonstration layout for incoming first-year engineering students.

---

# 📂 Repository Structure

```text
Haptics_Study/
│
├── 1_kHz_haptic_control_loop/
│   └── 1_kHz_haptic_control_loop.ino
│
├── Direct_Port_Manipulation/
│   └── Direct_Port_Manipulation.ino
│
├── Dual_Rate_FO-SLS_model/
│   └── Dual_Rate_FO-SLS_model.ino
│
├── Fractional_Order_Kelvin_Voigt_Model/
│   ├── Fractional_Order_Kelvin_Voigt_Model.ino
│   ├── graphs.py
│   └── haptic_plotter.py
│
├── Fractional_Order_SLS-Model/
│   ├── Fractional_Order_SLS-Model.ino
│   └── simulation_test.py
│
├── LMD18200T_Code/
│   └── LMD18200T_Code.ino
│
├── MD10C_R3_Code/
│   └── MD10C_R3_Code.ino
│
├── 3d-drawings/
│   └── Mechanical CAD (.stl) files
│
├── Paper and Notes/
│   └── Research papers, technical reports, lecture notes
│
├── haptic_with_noise.ino
└── README.md
```

---

# 📁 Project Overview

## 1️⃣ 1_kHz_haptic_control_loop

The primary production implementation.

**Features**

- Deterministic **1 kHz real-time control loop**
- Native **32-bit integer arithmetic**
- Embedded **2nd-order Butterworth low-pass filter**
- Passivity constraint protection
- Downsampled telemetry
- Optimized for the **L298N** motor driver

This serves as the baseline implementation for the remainder of the repository.

---

## 2️⃣ Direct_Port_Manipulation

Experimental implementation replacing Arduino API calls with direct manipulation of AVR hardware registers.

**Goals**

- Reduced instruction latency
- Lower interrupt overhead
- Faster GPIO transitions
- Improved deterministic timing

---

## 3️⃣ Fractional_Order_SLS_Model

Implements a **Fractional-Order Standard Linear Solid (FO-SLS)** virtual wall model.

Unlike a traditional spring-damper wall, this model captures viscoelastic behavior using fractional calculus, allowing more realistic rendering of compliant materials.

Includes simulation utilities for validating model behavior.

---

## 4️⃣ Fractional_Order_Kelvin_Voigt_Model

Implements a **Fractional Kelvin-Voigt** viscoelastic virtual environment.

Includes Python utilities for:

- Force visualization
- Experimental plotting
- Offline analysis

---

## 5️⃣ Dual_Rate_FO-SLS_Model

Experimental implementation combining:

- Dual-rate sampling
- Fractional-order viscoelastic rendering

to investigate stability improvements under computational constraints.

---

## 6️⃣ LMD18200T_Code

Version of the controller adapted for the **LMD18200T** high-current motor driver.

Designed for applications requiring:

- Higher continuous current
- Improved efficiency
- Industrial-grade motor control

---

## 7️⃣ MD10C_R3_Code

Alternative implementation supporting the **Cytron MD10C R3** motor driver.

Provides compatibility with PWM/Direction control architectures.

---

## 8️⃣ haptic_with_noise.ino

Legacy implementation preserved for educational comparison.

Characteristics:

- Floating-point computations
- Backward-difference velocity estimation
- Unrestricted serial logging
- Unstable loop timing

Useful for demonstrating common real-time control pitfalls.

---

## 9️⃣ 3d-drawings

Contains all mechanical CAD models used throughout development.

Includes multiple revisions of:

- Motor couplers
- Encoder couplers
- Shaft assemblies
- Handle assemblies
- Mounting structures

The designs evolved toward minimizing backlash while maximizing torsional rigidity.

---

## 📚 Paper and Notes

A curated collection of:

- Journal papers
- Conference publications
- Thesis material
- Technical reports
- Engineering notes

covering:

- Haptic rendering
- Fractional-order control
- Dual-rate systems
- Virtual wall stability
- Motor control
- Wave-variable haptics
- Embedded control

These documents served as the primary research references throughout development.

---

# ⚡ Hardware Configuration (Baseline L298N System)

## Power Supply

**MIMY RoHS S-120-12**

Provides:

- 12 V DC
- 10 A maximum current

| Terminal | Connection |
|----------|------------|
| L | AC Live |
| N | AC Neutral |
| Earth | Safety Ground |
| -V | L298N Ground |
| +V | L298N +12V |

---

## L298N Driver

| Pin | Connection |
|------|------------|
| 12V | Power Supply +12V |
| GND | Common Ground (Power Supply + Arduino) |
| 5V | Not Connected |
| ENA | Arduino Pin 5 (PWM) |
| IN1 | Arduino Pin 4 |
| IN2 | Arduino Pin 7 |
| OUT1 / OUT2 | DC Motor |

PWM is applied through **ENA**, while **IN1/IN2** determine motor direction.

---

## Optical Quadrature Encoder

Base resolution:

**400 PPR**

Effective resolution:

**1600 Counts Per Revolution (4× decoding)**

| Encoder | Arduino |
|----------|----------|
| VCC | 5V |
| GND | GND |
| Phase A | Pin 2 (Interrupt 0) |
| Phase B | Pin 3 (Interrupt 1) |

---

# 🧠 Engineering Development Journey

## Phase 1 — Eliminating the Serial Bottleneck

High-frequency `Serial.print()` operations saturated the UART transmit buffer, reducing the control loop from **1 kHz** to approximately **400 Hz**.

**Solution**

- Decoupled telemetry from control calculations
- Downsampled serial output to **40 Hz**
- Maintained deterministic **1 kHz** execution

---

## Phase 2 — Correct PWM Architecture

Early implementations modulated PWM through direction pins, producing asymmetric motor behavior and excessive dynamic braking.

**Solution**

- Fixed digital direction lines (`IN1`, `IN2`)
- PWM exclusively through `ENA`

---

## Phase 3 — Velocity Filtering

Backward-difference differentiation amplified encoder quantization noise.

**Solution**

A second-order Butterworth IIR low-pass filter configured with:

- Sampling frequency: **1000 Hz**
- Cutoff frequency: **100 Hz**

This significantly reduced quantization artifacts while preserving low phase delay.

---

## Phase 4 — Integer Arithmetic Optimization

The ATmega328P lacks hardware floating-point support.

The controller was rewritten entirely in native integer count space, reducing computational overhead and improving deterministic execution.

---

## Phase 5 — Transparency vs Stability

Adding ambient damping improved stability but reduced free-space transparency.

The production implementation therefore omits background damping, relying instead on:

- Optimized controller gains
- Mechanical damping
- Rigid zero-backlash transmission

to preserve a natural haptic experience.

---

# 🛠️ Virtual Wall Model

The unilateral virtual wall is represented by a discrete spring-damper model:

$$
F_{\text{wall}} = -(K \Delta x + B\,v_{\text{filtered}})
$$

where force is generated only after the handle penetrates the virtual wall boundary.

### Baseline Parameters

| Parameter | Value |
|-----------|-------|
| Wall Position | 400 counts (90°) |
| Stiffness | 22.5 PWM/count |
| Damping | 0.09 PWM/count/s |
| Maximum PWM | 200 |

---

# 🎯 Research Topics Covered

This repository explores multiple aspects of embedded haptic rendering, including:

- Classical virtual wall rendering
- Fractional-order viscoelastic modeling
- Standard Linear Solid (SLS) models
- Kelvin-Voigt models
- Dual-rate haptic rendering
- Direct AVR register manipulation
- Embedded real-time optimization
- Motor driver comparison (L298N, MD10C, LMD18200T)
- Mechanical transmission design
- Stability and passivity analysis
- Digital filtering
- Low-latency control systems

---

# 📄 License

This repository is intended for educational, research, and instructional use. Contributions and experimentation are encouraged.
