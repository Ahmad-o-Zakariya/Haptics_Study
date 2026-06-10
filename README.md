# Haptics_Study

A high-performance, 1-DOF (Degree of Freedom) haptic virtual wall rendering system developed on an 8-bit AVR architecture (Arduino Uno). This system implements a closed-loop force-feedback control system utilizing a high-resolution optical quadrature encoder, an industrial metal-cased switching power supply, and an L298N H-bridge motor driver.

Following successful validation and technical review by the senior faculty, this project has been officially approved as the benchmark instructional demonstration layout for incoming first-year engineering students next semester.

---

## 📂 Repository Architecture

* **`1_kHz_haptic_control_loop.ino`** – The production-grade control script. Features a deterministic 1 kHz real-time execution loop, native 32-bit integer count processing to eliminate floating-point emulation overhead, an embedded 2nd-order digital Butterworth low-pass filter, downsampled telemetry pacing, and a passivity constraint guard.
* **`haptic_with_noise.ino`** – Legacy codebase preserved for comparative study. Contains unbuffered floating-point math, raw backward-difference velocity differentiation, and unrestricted serial logging that triggers execution bottlenecks.
* **`Coupler-Shaft.stl` / `Coupler-Shaft-1.stl`** – Iterative 3D CAD models for the structural transmission coupler. Optimized for an interference fit to achieve absolute zero-backlash alignment between the motor output shaft, encoder housing, and manual user handle.
* **`Base_coupler_shaft.stl` / `Base_coupler_shaft_1.stl`** – Iterative design files for the mechanical structural base and shaft indexing assembly.

---

## ⚡ Hardware Wiring & Blueprint Configuration

### 1. High-Voltage Power Supply (MIMY RoHS S-120-12)
Provides an isolated +12V DC rail capable of sourcing up to 10A. Massive current headroom prevents transient voltage drops during peak unilateral constraint saturation (when the motor actively opposes high manual torque).

* **Terminal 1 (L)** → AC Live Line (Mains Power input)
* **Terminal 2 (N)** → AC Neutral Line (Mains Power input)
* **Terminal 3 (Earth Ground)** → AC Safety Ground Connection
* **Terminal 4 (-V / COM)** → Tied directly to the L298N Driver Ground Terminal
* **Terminal 6 (+V)** → Tied directly to the L298N Driver +12V Input Terminal

### 2. L298N H-Bridge Driver (Instructor-Approved Split-Control Layout)
Configured using an explicit hardware separation layout: digital switches govern directional polarity while a dedicated hardware timer channel throttles the voltage magnitude via Pulse Width Modulation (PWM).

* **12V Terminal** → Connected to Terminal 6 (+12V DC) of the power supply.
* **GND Terminal** → **CRITICAL COMMON GROUND BRIDGE.** Clamps two distinct lines: One running to Terminal 4 (COM) of the power supply, and one running directly to an Arduino system `GND` pin.
* **5V Terminal** → **Left Disconnected.** The microcontroller logic rail runs independently via USB to prevent cross-rail electrical fighting.
* **ENA Pin** → *Onboard black plastic jumper cap removed completely.* Connected directly to Arduino **Pin 5** (Hardware PWM).
* **IN1 Pin** → Connected to Arduino **Pin 4** (Digital Output).
* **IN2 Pin** → Connected to Arduino **Pin 7** (Digital Output).
* **OUT1 / OUT2 Terminals** → Connected directly to the brushed DC motor terminal leads.

### 3. Optical Quadrature Encoder
Yields a base resolution of 400 Pulses Per Revolution (PPR), multiplied to an effective resolution of 1600 Counts Per Revolution (CPR) via 4x Grey-code state tracking.

* **VCC Wire** → Connected to Arduino **5V** rail.
* **GND Wire** → Connected to Arduino **GND** rail.
* **Phase A** → Connected to Arduino **Pin 2** (Dedicated Hardware Interrupt 0).
* **Phase B** → Connected to Arduino **Pin 3** (Dedicated Hardware Interrupt 1).

---

## 🧠 Core Engineering Insights & Control Journey

### Phase 1: Overcoming the Telemetry Buffer Trap
Initial testing revealed that running high-frequency serial transmissions (`Serial.print`) inside a fast timing window causes immediate buffer saturation. At 115200 baud, transmitting a 22-character string takes approximately 1.91ms—instantly breaking the 1.0ms sample frame requirement. This turned `Serial.print` into a blocking function, dropping the loop frequency to an unstable ~400 Hz and inducing severe auditory buzzing and mechanical chatter. 
* **The Fix:** Telemetry data is completely separated from control loop calculations. A downsampling counter isolates serial execution, executing output routines strictly every 25ms (40 Hz refresh rate) while the hardware loop runs deterministically at 1000 Hz.

### Phase 2: Resolving Driver Asymmetry & Dynamic Braking Losses
Early code layouts attempted to control speed via an individual input line while cycling the secondary input pin between states. In an L298N H-bridge, this causes asymmetric motor properties: alternating between forward-drive and high-frequency dynamic braking rather than forward-drive and coasting. This imbalance led to rapid motor coil overheating and inconsistent force sensations.
* **The Fix:** Implemented the classic mechatronic standard. Direction lines (`IN1`, `IN2`) operate as solid digital rails, establishing a clean, steady voltage path. The `ENA` pin receives the continuous high-speed PWM stream, functioning as a true throttle valve to scale voltage linearly.

### Phase 3: Attenuating Derivative Quantization Noise
Deriving instantaneous velocity ($v = \Delta x / \Delta t$) via discrete backward differences introduces extreme high-frequency quantization noise due to the finite step resolution of the encoder. This noise, when multiplied by a dampening scalar ($B$), manifests as an erratic, coarse texture on the motor handle.
* **The Fix:** Cascaded a digital 2nd-order Infinite Impulse Response (IIR) Butterworth Low-Pass Filter directly into the velocity derivation pipeline. Configured for a cutoff frequency ($f_c$) of 100 Hz at a sampling rate ($f_s$) of 1000 Hz, the filter strips away structural quantization harmonics while maintaining an exceptionally low phase delay of just 6.4ms, preserving immediate haptic transparency.

### Phase 4: Integer Count Optimization for 8-bit Microcontrollers
The ATmega328P processor lacks a hardware Floating Point Unit (FPU). Processing physics equations in float space (converting ticks into degrees) forces the compiler to mimic decimal math using extensive software routines, consuming valuable clock cycles and risking loop overrun.
* **The Fix:** The entire loop was rewritten to execute in native 32-bit integer count space. The virtual wall is placed at a precise 1/4 turn boundary ($90^\circ = 400$ raw counts). System positioning and differential velocity steps are evaluated entirely via native integer assembly instructions, maximizing computational efficiency.

### Phase 5: Passivity Constraints and Free-Space Transparency
To counter a violent bounce-back effect caused by the rapid conversion of stored potential energy when releasing the handle inside the wall, adding artificial background damping (`B_AMBIENT`) was evaluated. While stable on paper, physical interaction tests revealed that global ambient damping destroyed "haptic transparency"—creating an artificial fluid resistance in open air.
* **The Fix:** `B_AMBIENT` was intentionally omitted from the production code. By maintaining a completely uninhibited free-space zone, the device achieves absolute transparency. High-frequency tracking stability and structural damping are handled perfectly by the physical characteristics of the customized, zero-backlash transmission coupler.

---

## 🛠️ Performance Parameters & Physics Formulation

The haptic environment models a structural unilateral constraint using a discrete Spring-Damper representation. The control torque command is formulated when the raw count position breaches the unilateral boundary:

$$\text{If } x_{\text{current}} > x_{\text{wall}} \implies F_{\text{wall}} = -(K \cdot \Delta x + B \cdot v_{\text{filtered}})$$

Where parameters are mathematically scaled directly to match native count spatial profiles:

* **Virtual Wall Position ($x_{\text{wall}}$):** 400 Counts (Exactly $90^\circ$ tracking boundary)
* **Stiffness ($K$):** 22.5 PWM Units per Count (Equivalent to 100.0 PWM/Degree)
* **Damping ($B$):** 0.09 PWM Units per Count/Second (Equivalent to 0.4 PWM/Degree/Sec)
* **Safety Saturation Limit ($PWM_{\text{max}}$):** 200 (Protects motor windings from thermal saturation)
