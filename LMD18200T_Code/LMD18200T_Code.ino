/**
 * 1-DOF Cascaded Current-Controlled Haptic System
 * Hardware: Arduino Uno R4 Minima, LMD18200T, RS-555PH
 * Architecture: 1kHz Outer Haptic Loop -> Inner PI Current Loop
 */

// --- Pin Definitions ---
const int clkPin = 2;   // Encoder Phase A
const int dtPin = 3;    // Encoder Phase B
const int DIR_PIN = 4;  // LMD18200T Direction (Pin 3)
const int PWM_PIN = 5;  // LMD18200T PWM (Pin 5)
const int CURRENT_SENSE_PIN = A0; // LMD18200T Current Sense (Pin 8) via RC Filter

// --- Encoder Variables ---
volatile long encoderPos = 0;
volatile int lastEncoded = 0;

// --- Timing & Loop Control ---
unsigned long lastTimeMicros = 0;
long lastRawPosition = 0; 
const unsigned long SAMPLE_INTERVAL_MICROS = 1000; // 1 millisecond (1 kHz loop)
int telemetryCounter = 0;

// --- 2nd-Order Butterworth Filter (fc = 100Hz, fs = 1000Hz) ---
const float b0 = 0.067455;
const float b1 = 0.134911;
const float b2 = 0.067455;
const float a1 = -1.142981;
const float a2 = 0.412802;

float x_hist[2] = {0.0, 0.0};
float y_hist[2] = {0.0, 0.0};

// --- Hardware Constants ---
const float RS_OHMS = 2200.0;       // 2.2k Ohm sense resistor
const float SENSE_RATIO = 0.000377; // 377 uA per Amp

// --- PI Current Controller Parameters ---
float Kp_curr = 25.0;     // Proportional gain
float Ki_curr = 500.0;    // Integral gain
float integralError = 0.0;
const float MAX_INTEGRAL = 200.0; // Anti-windup limit

// --- Haptic Physics Parameters ---
const long WALL_POSITION_COUNTS = 400;  
const float K_STIFFNESS = 0.05;  // Output is now in Amps, keep this low initially!
const float B_DAMPING = 0.002; 
// --- SAFETY LIMITS ---
const float MAX_CURRENT_AMPS = 0.3; // Limit to 0.3A to save the capacitor
const int MAX_PWM_LIMIT = 100;      // Limit PWM to 100 (out of 255)

void setup() {
  // Configure LMD18200T Logic Pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  // Configure Encoder Pins
  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dtPin, INPUT_PULLUP);
  
  // Uno R4 allows higher ADC resolution for cleaner current sensing
  analogReadResolution(12); // Sets analogRead range to 0-4095

  // Initialize Encoder State
  int MSB = digitalRead(clkPin); 
  int LSB = digitalRead(dtPin); 
  lastEncoded = (MSB << 1) | LSB;

  // Attach Interrupts for 4x Quadrature Decoding
  attachInterrupt(digitalPinToInterrupt(clkPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dtPin), updateEncoder, CHANGE);

  Serial.begin(115200);
  lastTimeMicros = micros();
}

void loop() {
  unsigned long currentTimeMicros = micros();
  
  // Enforce strict 1 millisecond execution clock
  if (currentTimeMicros - lastTimeMicros >= SAMPLE_INTERVAL_MICROS) {
    long currentRawPos;
    
    // Atomic Read: Safely snapshot encoder ticks
    noInterrupts();
    currentRawPos = encoderPos;
    interrupts();

    // 1. Calculate and Filter Velocity
    float dt = (currentTimeMicros - lastTimeMicros) / 1000000.0;
    long deltaCounts = currentRawPos - lastRawPosition; 
    float rawVelocityCounts = (float)deltaCounts / dt;

    float filteredVelocityCounts = (b0 * rawVelocityCounts) + (b1 * x_hist[0]) + (b2 * x_hist[1]) 
                                 - (a1 * y_hist[0]) - (a2 * y_hist[1]);

    // Shift discrete time delay history registers
    x_hist[1] = x_hist[0];     
    x_hist[0] = rawVelocityCounts;   
    y_hist[1] = y_hist[0];     
    y_hist[0] = filteredVelocityCounts;

    // 2. Outer Loop: Haptic Virtual Wall -> Target Current
    float targetCurrentAmps = 0.0;
    
    if (currentRawPos > WALL_POSITION_COUNTS) {
      long penetration = currentRawPos - WALL_POSITION_COUNTS;
      float springForce = K_STIFFNESS * penetration;
      float dampForce = B_DAMPING * filteredVelocityCounts;
      
      // Calculate opposing force
      targetCurrentAmps = -(springForce + dampForce);
      
      // Passivity guard: Wall pushes out, it never pulls you in
      if (targetCurrentAmps > 0.0) targetCurrentAmps = 0.0;
    } else {
      // Free space
      targetCurrentAmps = 0.0;
      integralError = 0.0;
    }

    // Clamp target current for motor/driver safety
    if (targetCurrentAmps < -MAX_CURRENT_AMPS) targetCurrentAmps = -MAX_CURRENT_AMPS;
    if (targetCurrentAmps > MAX_CURRENT_AMPS) targetCurrentAmps = MAX_CURRENT_AMPS;

    // 3. Inner Loop: PI Current Controller
    float actualCurrentAmps = readMotorCurrent();
    
    // Calculate error (comparing magnitudes)
    float currentError = abs(targetCurrentAmps) - actualCurrentAmps;
    
    // Proportional Term
    float pTerm = Kp_curr * currentError;
    
    // Integral Term with Anti-Windup
    integralError += currentError * dt;
    if (integralError > MAX_INTEGRAL) integralError = MAX_INTEGRAL;
    if (integralError < -MAX_INTEGRAL) integralError = -MAX_INTEGRAL;
    float iTerm = Ki_curr * integralError;
    
    // Calculate final PWM control effort (0-255 mapped from PI output)
    float controlEffort = pTerm + iTerm;
    
    int pwmOutput = (int)controlEffort;
    
    // Clamp PWM to 8-bit bounds
    if (pwmOutput > 255) pwmOutput = 255;
    if (pwmOutput < 0) pwmOutput = 0;

    // 4. Actuate H-Bridge
    // Direction logic based on the sign of targetCurrentAmps
    if (pwmOutput > MAX_PWM_LIMIT) pwmOutput = MAX_PWM_LIMIT; // Apply the safe limit
    if (targetCurrentAmps < 0) {
      digitalWrite(DIR_PIN, LOW); // Entering wall (pushing back)
    } else {
      digitalWrite(DIR_PIN, HIGH);
    }
    analogWrite(PWM_PIN, pwmOutput);

    // 5. Downsampled Telemetry Monitor (Runs at 40Hz)
    telemetryCounter++;
    if (telemetryCounter >= 25) {
      Serial.print("Pos:"); 
      Serial.print(currentRawPos);
      Serial.print("\tTarget_A:"); 
      Serial.print(targetCurrentAmps, 3);
      Serial.print("\tActual_A:"); 
      Serial.print(actualCurrentAmps, 3);
      Serial.print("\tPWM:"); 
      Serial.println(pwmOutput);
      
      telemetryCounter = 0;
    }

    // Cache state variables for the next iteration step
    lastRawPosition = currentRawPos;
    lastTimeMicros = currentTimeMicros;
  }
}

/**
 * Reads the analog voltage from the LMD18200T current sense pin
 * and converts it into actual motor current in Amperes.
 */
float readMotorCurrent() {
  // Read 12-bit ADC (0-4095 corresponds to 0-5V)
  int rawADC = analogRead(CURRENT_SENSE_PIN);
  float voltage = (rawADC / 4095.0) * 5.0;
  
  // Imotor = V_ADC / (Rs * 377uA)
  float currentAmps = voltage / (RS_OHMS * SENSE_RATIO);
  
  // Quick safety guard to prevent tiny ADC noise from registering as current
  if (currentAmps < 0.25) currentAmps = 0.0;
  
  return currentAmps;
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