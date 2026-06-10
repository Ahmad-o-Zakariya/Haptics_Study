/**
 * 1-DOF Haptic Virtual Wall Rendering System (Integer Count Optimized)
 * Standard Textbook Layout: ENA = PWM Speed, IN1/IN2 = Digital Direction
 * Parameters: Wall at 90 degrees (400 counts), High Stiffness, Max PWM 200
 */

// --- Encoder Configuration (Orange 400 PPR -> 1600 CPR in 4x Mode) ---
const int COUNTS_PER_REV = 1600;

// Hardware Interrupt Pins
const int clkPin = 2;
const int dtPin = 3;

// Actuator Pin Definitions (Standard L298N Architecture)
const int ENA_PIN = 5;  // Outputs 0-255 PWM to regulate force magnitude
const int IN1_PIN = 4;  // Digital Direction Line 1
const int IN2_PIN = 7;  // Digital Direction Line 2

// Volatile variables used across ISR and main loop (Pure Integers)
volatile long encoderPos = 0;
volatile int lastEncoded = 0;

// High-Speed Execution Control Loop Parameters (1 kHz Loop Execution)
unsigned long lastTimeMicros = 0;
long lastRawPosition = 0; // Native integer tracking to eliminate FPU overhead
const unsigned long SAMPLE_INTERVAL_MICROS = 1000; // Exact 1ms frame time

// Telemetry Pacing Counter (Downsamples serial output to prevent transmission lag)
int telemetryCounter = 0;

// --- 2nd-Order Butterworth Filter Coefficients (fc = 100Hz, fs = 1000Hz) ---
const float b0 = 0.067455;
const float b1 = 0.134911;
const float b2 = 0.067455;
const float a1 = -1.142981;
const float a2 = 0.412802;

float x_hist[2] = {0.0, 0.0};
float y_hist[2] = {0.0, 0.0};

// --- Haptic Virtual Wall Environment Parameters (CONVERTED TO RAW COUNTS) ---
const long WALL_POSITION_COUNTS = 400;  // 90 degrees explicitly translated to a 1/4 turn 
const float K_STIFFNESS = 22.5;         // Scaled: Maps to your original 100.0 PWM/Degree
const float B_DAMPING = 0.09;           // Scaled: Maps to your original 0.4 PWM/(Deg/Sec)

// **FULL STIFFNESS CEILING** -> Maximum safe power threshold for rigid rendering
const int MAX_PWM_OUTPUT = 200;         

void setup() {
  // Configure Encoder Pins with Internal Pullups
  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dtPin, INPUT_PULLUP);
  
  // Configure L298N Pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  
  // Initialize driver to a completely safe, unpowered state
  analogWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  // Read initial quadrature state
  int MSB = digitalRead(clkPin);
  int LSB = digitalRead(dtPin);
  lastEncoded = (MSB << 1) | LSB;

  // Attach Hardware Interrupts to both edges for high-precision 4x decoding
  attachInterrupt(digitalPinToInterrupt(clkPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dtPin), updateEncoder, CHANGE);

  // Open fast hardware serial stream
  Serial.begin(115200);
  lastTimeMicros = micros();
}

void loop() {
  unsigned long currentTimeMicros = micros();
  
  // Enforce strict 1 millisecond execution clock
  if (currentTimeMicros - lastTimeMicros >= SAMPLE_INTERVAL_MICROS) {
    long currentRawPos;

    // Atomic Read: Safely snapshot encoder ticks without interrupt corruption
    noInterrupts();
    currentRawPos = encoderPos;
    interrupts();

    // 1. Compute Instantaneous Real-Time Velocity purely via count steps
    float dt = (currentTimeMicros - lastTimeMicros) / 1000000.0;
    long deltaCounts = currentRawPos - lastRawPosition; // Instant native subtraction
    float rawVelocityCounts = (float)deltaCounts / dt;

    // 2. Process signal through the 2nd-Order Digital Butterworth Filter
    float filteredVelocityCounts = (b0 * rawVelocityCounts) + (b1 * x_hist[0]) + (b2 * x_hist[1]) 
                                 - (a1 * y_hist[0]) - (a2 * y_hist[1]);

    // Shift discrete time delay history registers
    x_hist[1] = x_hist[0];     
    x_hist[0] = rawVelocityCounts;   
    y_hist[1] = y_hist[0];     
    y_hist[0] = filteredVelocityCounts;

    // 3. Haptic Environment Physics Execution (The Wall)
    float motorForceCmd = 0.0;

    // Boundary Evaluation utilizing native high-speed integer comparison
    if (currentRawPos > WALL_POSITION_COUNTS) {
      // Calculate depth of penetration using raw counts
      long penetrationDepthCounts = currentRawPos - WALL_POSITION_COUNTS;

      // Compute opposing Hookean Spring force and Viscous Damping force
      float springForce = K_STIFFNESS * penetrationDepthCounts;
      float dampingForce = B_DAMPING * filteredVelocityCounts;
      
      // Unilateral Constraint Law: Force pushes back against user penetration
      motorForceCmd = -(springForce + dampingForce);

      // Passivity Guard: Ensure the wall never "pulls" the hand during manual exit
      if (motorForceCmd > 0.0) {
        motorForceCmd = 0.0;
      }
    } else {
      // Free Space: No physical constraints applied outside the wall boundary
      motorForceCmd = 0.0;
    }

    // 4. Run low-level hardware actuation bridge
    actuateMotor(motorForceCmd);

    // 5. Downsampled Telemetry Monitor Loop (Runs at 40Hz to prevent serial blocking)
    telemetryCounter++;
    if (telemetryCounter >= 25) {
      Serial.print("CountPos:");   Serial.print(currentRawPos);
      Serial.print(",CountVel:");  Serial.print(filteredVelocityCounts, 1);
      Serial.print(",ForceCmd:");  Serial.println(motorForceCmd, 1);
      telemetryCounter = 0;
    }

    // Cache state variables for the next iteration step
    lastRawPosition = currentRawPos;
    lastTimeMicros = currentTimeMicros;
  }
}

/**
 * Standard L298N Actuation Function
 * Sets digital pins for direction state, then pipes PWM power straight to ENA
 */
void actuateMotor(float forceCommand) {
  // Convert continuous fractional values into clean 8-bit discrete integers
  int pwmValue = abs((int)forceCommand);
  
  // Enforce safety ceiling constraints to protect your hardware components
  if (pwmValue > MAX_PWM_OUTPUT) {
    pwmValue = MAX_PWM_OUTPUT;
  }
  
  // State 1: Determine direction by setting digital pins HIGH/LOW combinations
  if (forceCommand > 0) {
    // Clockwise direction assignment
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } 
  else if (forceCommand < 0) {
    // Counter-Clockwise direction assignment
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } 
  else {
    // Total stop state
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  // State 2: Feed the PWM duty cycle value directly into the ENA throttle line
  analogWrite(ENA_PIN, pwmValue);
}

/**
 * High-Speed 4x Mode Quadrature Decoder State Machine
 */
void updateEncoder() {
  int MSB = digitalRead(clkPin); 
  int LSB = digitalRead(dtPin);  

  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded; 

  if(sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderPos++;
  if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderPos--;

  lastEncoded = encoded; 
}