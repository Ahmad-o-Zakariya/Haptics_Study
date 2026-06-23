/**
 * ============================================================================
 * HIGH-PERFORMANCE 1-DOF HAPTIC VIRTUAL WALL CONTROL SYSTEM (SI CONVERTED)
 * Architecture: Bare-Metal Renesas RA4M1 (ARM Cortex-M4 @ 48 MHz)
 * Hardware Target: Arduino Uno R4 Minima + L298N H-Bridge Driver
 * * Hardware Mapping:
 * - Encoder Channel A (CLK): Arduino Pin 2 -> Native P301 (Port 3, Pin 1)
 * - Encoder Channel B (DT):  Arduino Pin 3 -> Native P302 (Port 3, Pin 2)
 * - Motor Driver Enable (PWM):Arduino Pin 5 -> Native P102 (Port 1, Pin 2) [GPT2]
 * - Motor Driver Direction 1: Arduino Pin 4 -> Native P111 (Port 1, Pin 11)
 * - Motor Driver Direction 2: Arduino Pin 7 -> Native P103 (Port 1, Pin 3)
 * ============================================================================
 */

// --- PHYSICAL HARDWARE CONFIGURATION ---
const float ENCODER_PPR = 400.0f;
const float ENCODER_CPR = ENCODER_PPR * 4.0f;       // 1600 Counts/Rev via 4x decoding
const float TWO_PI = 6.28318530f;

// --- GLOBAL MECHATRONIC TUNING PARAMETERS (SI UNITS) ---
const uint32_t PWM_FREQUENCY_HZ = 12000;            // Optimal range for L298N: 8000 Hz - 16000 Hz
const float WALL_THRESHOLD_RAD = 1.57079632f;       // Wall boundary at exactly 90 degrees (pi/2 radians)
const float K_STIFFNESS_SI = 8.5f;                  // Virtual Spring Stiffness (Newton-meters per Radian)
const float B_DAMPING_SI = 0.08f;                   // Virtual Damping Coefficient (Newton-meter-seconds per Radian)
const float MAX_CONTROL_TORQUE = 0.35f;             // Peak torque scaling factor for your motor setup (N-m)
const int MAX_PWM_OUTPUT = 255;                     // Bound for 8-bit user scaling logic
const bool INVERT_MOTOR_DIRECTION = true;           // Software toggle to correct force direction

// --- VOLATILE SHARED INTERRUPT VARIABLES ---
volatile long encoderPos = 0;
volatile int lastEncoded = 0;

// --- CONTROL LOOP TIMING SYSTEM ---
unsigned long lastTimeMicros = 0;
long lastRawPosition = 0; 
const unsigned long SAMPLE_INTERVAL_MICROS = 1000;  // Fixed 1 kHz Discrete Execution Loop
int telemetryCounter = 0;

// --- DIGITAL FILTERING COEFFICIENTS (2nd-Order Butterworth @ 1kHz) ---
const float b0 = 0.067455, b1 = 0.134911, b2 = 0.067455;
const float a1 = -1.142981, a2 = 0.412802;
float x_hist[2] = {0.0, 0.0};                       // Input velocity history registers [n-1, n-2]
float y_hist[2] = {0.0, 0.0};                       // Output filtered velocity history registers [n-1, n-2]

// --- HARDWARE TIMER CALCULATED CONSTANTS ---
uint32_t timerPeriodCounts = 0;                     // Stores dynamic GTPR overflow ceiling

// Forward Declarations
void actuateMotorL298NRaw(float forceCommand);
void updateEncoderRegisterMode();

void setup() {
  // Calculate hardware counting ceiling based on the 48 MHz peripheral clock
  timerPeriodCounts = 48000000 / PWM_FREQUENCY_HZ;

  // 1. CONFIGURE DIGITAL OUTPUTS (Port 1 Direction Register)
  // Turn Pin 4 (P111) and Pin 7 (P103) into Outputs
  R_PORT1->PDR |= (1 << 11) | (1 << 3);

  // 2. CONFIGURE DIGITAL INPUTS (Port 3 Direction Register)
  // Clear Bits 1 and 2 of Port 3 to establish inputs for Pins 2 and 3
  R_PORT3->PDR &= ~((1 << 1) | (1 << 2));
  
  // Enable internal Pull-Up Resistors via the Pin Function Select (PFS) registers
  R_PFS->PORT[3].PIN[1].PmnPFS_b.PCR = 1; // Pin 2 Pull-Up
  R_PFS->PORT[3].PIN[2].PmnPFS_b.PCR = 1; // Pin 3 Pull-Up

  // 3. ROUTE PERIPHERAL HARDWARE TIMER TO PIN 5
  // Route Pin 5 (P102) to General Purpose Timer 2 (GPT2) output
  R_PFS->PORT[1].PIN[2].PmnPFS_b.PSEL = 0b00011; // Assign function to GPT
  R_PFS->PORT[1].PIN[2].PmnPFS_b.PMR  = 1;       // Activate peripheral mode on pin

  // 4. CONFIGURE GENERAL PURPOSE TIMER 2 (GPT2) REGISTER MAP
  R_GPT2->GTCR_b.CST = 0;       // Stop timer during initialization configuration
  R_GPT2->GTCR_b.MD = 0b000;    // Select Sawtooth wave mode (Periodic counting)
  R_GPT2->GTCR_b.TPCS = 0b000;  // Use undivided master clock frequency (48 MHz)
  R_GPT2->GTPR = timerPeriodCounts; // Inject calculated overflow ceiling into GTPR
  R_GPT2->GTCCR[1] = 0;         // Start with clear Compare Register (0% initial duty cycle)
  R_GPT2->GTIOR = 0x00001F00;   // Force output pin HIGH on cycle start, LOW on match
  R_GPT2->GTCNT = 0;            // Zero out current stopwatch count
  R_GPT2->GTCR_b.CST = 1;       // Arm and start GPT2 hardware timer instantly

  // 5. INITIALIZE QUADRATURE ENCODER STATE SENSING
  uint32_t initialPort3 = R_PORT3->PIDR;
  int initialMSB = (initialPort3 >> 1) & 0x01; 
  int initialLSB = (initialPort3 >> 2) & 0x01; 
  lastEncoded = (initialMSB << 1) | initialLSB;

  // Attach execution triggers to standard hardware pin vectors (Ensures full 4x edge catching)
  attachInterrupt(digitalPinToInterrupt(2), updateEncoderRegisterMode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), updateEncoderRegisterMode, CHANGE);

  Serial.begin(115200);
  lastTimeMicros = micros();
}

void loop() {
  unsigned long currentTimeMicros = micros();
  
  // Enforce rigid, deterministic 1 kHz control step interval
  if (currentTimeMicros - lastTimeMicros >= SAMPLE_INTERVAL_MICROS) {
    long currentRawPos;

    // ATOMIC GUARD SEGMENT: Safeguard shared volatile data against race conditions
    __disable_irq();
    currentRawPos = encoderPos;
    __enable_irq();

    // Calculate sample timeframe delta (dt = 0.001s)
    float dt = (float)SAMPLE_INTERVAL_MICROS / 1000000.0f; 
    
    // PHYSICAL STATE TRANSFORMATION: Convert raw encoder values directly to Radians
    float theta = ((float)currentRawPos / ENCODER_CPR) * TWO_PI;

    // Calculate raw velocity in Radians per Second
    long deltaCounts = currentRawPos - lastRawPosition; 
    float rawVelocityRadSec = (((float)deltaCounts / ENCODER_CPR) * TWO_PI) / dt;

    // DIGITAL SIGNAL PROCESSING: 2nd-Order Butterworth Filtered Velocity
    float filteredVelocityRadSec = (b0 * rawVelocityRadSec) + (b1 * x_hist[0]) + (b2 * x_hist[1]) 
                                 - (a1 * y_hist[0]) - (a2 * y_hist[1]);
    
    // Shift execution histories forward
    x_hist[1] = x_hist[0];             x_hist[0] = rawVelocityRadSec;   
    y_hist[1] = y_hist[0];             y_hist[0] = filteredVelocityRadSec;

    float wallTorque = 0.0f;

    // HAPTIC RENDERING BOUNDARY (Evaluated entirely in physical physics parameters)
    if (theta > WALL_THRESHOLD_RAD) {
      float penetrationDepthRad = theta - WALL_THRESHOLD_RAD;
      
      float springTorque = K_STIFFNESS_SI * penetrationDepthRad;
      float dampingTorque = B_DAMPING_SI * filteredVelocityRadSec;
      
      // Calculate resulting opposing force vector (Torque = -(K*x + B*v))
      wallTorque = -(springTorque + dampingTorque);
      
      // PASSIVITY GUARD CONSTRAINT: Force must only push back, never pull inward
      if (wallTorque > 0.0f) wallTorque = 0.0f; 
    }

    // ACTUATOR SCALING INTERFACE: Normalize physical torque into 8-bit PWM values
    float normalizedEffort = wallTorque / MAX_CONTROL_TORQUE;
    float wallForcePwm = normalizedEffort * (float)MAX_PWM_OUTPUT;

    // Pass force payload directly down to physical motor control registers
    actuateMotorL298NRaw(wallForcePwm);

    // High-Speed Serial Telemetry Output (Throttle execution rate to 40 Hz)
    telemetryCounter++;
    if (telemetryCounter >= 25) {
      int activePwm = abs((int)wallForcePwm);
      if (activePwm > MAX_PWM_OUTPUT) activePwm = MAX_PWM_OUTPUT;
      
      Serial.print("AngleRad:");    Serial.print(theta, 3);
      Serial.print(",VelRadSec:");  Serial.print(filteredVelocityRadSec, 2);
      Serial.print(",TorqueNm:");   Serial.print(wallTorque, 4);
      Serial.print(",DutyCycle:");  Serial.print(((float)activePwm / 255.0f) * 100.0f, 1);
      Serial.println("%");
      telemetryCounter = 0;
    }

    // Save cycle states for next loop execution
    lastRawPosition = currentRawPos;
    lastTimeMicros = currentTimeMicros;
  }
}

/**
 * Bare-Metal Motor Power Control Function
 * Accepts calculated force vector, sets H-Bridge switches, and dynamically scales PWM duty cycle.
 */
void actuateMotorL298NRaw(float forceCommand) {
  int pwmValue = abs((int)forceCommand);
  if (pwmValue > MAX_PWM_OUTPUT) pwmValue = MAX_PWM_OUTPUT;
  
  bool directionFlag = (forceCommand > 0);
  if (INVERT_MOTOR_DIRECTION) directionFlag = !directionFlag;

  // DIRECT PORT MANIPULATION: Safe Bitmasking execution via PODR registers
  if (pwmValue == 0) {
    // Engage H-Bridge Brake State by pulling both direction lines LOW simultaneously
    R_PORT1->PODR &= ~((1 << 11) | (1 << 3)); 
  } else if (directionFlag) {
    // Pin 4 (P111) HIGH, Pin 7 (P103) LOW
    R_PORT1->PODR |= (1 << 11);
    R_PORT1->PODR &= ~(1 << 3);
  } else {
    // Pin 4 (P111) LOW, Pin 7 (P103) HIGH
    R_PORT1->PODR &= ~(1 << 11);
    R_PORT1->PODR |= (1 << 3);
  }

  // PROPORTIONAL CEILING SCALING MATH: Translates 8-bit command into current timer period counts
  uint32_t compareValue = (uint32_t)((float)pwmValue * ((float)timerPeriodCounts / 255.0f));
  
  // Inject calculated duty tripwire register directly into GPT2 compare buffer channel
  R_GPT2->GTCCR[1] = compareValue; 
}

/**
 * Interrupt Service Routine: Handled at bare-metal execution speeds.
 * Rapidly reads Port 3 input pins to determine quadrature gray code state transitions.
 */
void updateEncoderRegisterMode() {
  // Sample a snapshot of all inputs on Port 3 in a single cycle
  uint32_t port3State = R_PORT3->PIDR;
  
  // Isolate Pin 1 (Bit 1) and Pin 2 (Bit 2) state via single-step bit shifting and masks
  int MSB = (port3State >> 1) & 0x01; 
  int LSB = (port3State >> 2) & 0x01; 
  
  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded; 
  
  // Evaluate Gray Code Quadrature State Machine Table (4x Decoded States)
  if(sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderPos++;
  if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderPos--;
  
  lastEncoded = encoded; 
}