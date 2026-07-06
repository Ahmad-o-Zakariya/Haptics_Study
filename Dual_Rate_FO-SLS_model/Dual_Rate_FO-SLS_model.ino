#include "pwm.h"

// Hardware Pin Configurations
const int CLK_PIN = 2;
const int DT_PIN = 3;
const int DIR_PIN = 4;
const int CURRENT_SEN_PIN = A0;

// Hardware Portability Configurations
const bool INVERT_MOTOR_DIRECTION = false; 

// Electromechanical Transformation Constants
const float INTERACTION_RADIUS = 0.001275f; 
const float MOTOR_KT = 0.020f;              
const float FORCE_TO_CURRENT = INTERACTION_RADIUS / MOTOR_KT; 

// ACS712 Current Sensing Architecture
const float ADC_VCC = 5.000f;
const float ADC_RESOLUTION = 1023.0f;
const float SENSITIVITY = 0.100f;           
float zero_voltage_baseline = 2.5113f;      

const float CONFIGURABLE_BETA = 0.15f; 
float filteredCurrent = 0.0f;

// Safety Limits
const float I_SAFE_LIMIT = 3.0f;           
const float I_CONTINUOUS_MAX = 1.50f;       
const unsigned long OVERCURRENT_TIMEOUT_MS = 350; 
unsigned long overcurrentTimer = 0;
bool overcurrentActive = false;
bool systemKilled = false;

// Current Loop PI Controller Configurations
const float Kp_curr = 45.0f;                
const float Ki_curr = 180.0f;               
float currentIntegralError = 0.0f;          
const float TS_FAST = 0.0005f;               // Fast Loop Period: 1ms (1000 Hz)

// ==========================================
// DUAL-RATE STRUCTURAL CONFIGURATIONS (Paper 1 & 2)
// ==========================================
const int N_RATIO = 5;                      // Down-sampling multiplier ratio (N)
const float TS_SLOW = TS_FAST * N_RATIO;    // Slow Loop Period: 5ms (200 Hz)
int slowLoopCounter = 0;

// FO-SLS Model System Parameters 
const float K_0 = 2.89f;
const float K_1 = 5.70f;
const float B_1 = 5.89f;
const float ALPHA = 0.7f;                   // Sweep from 0.1 to 1.0 to generate your graph

// Memory Buffers for Fractional Calculus (N = 101)
const int N_MEM = 101;
float errorHistory[N_MEM];
float forceHistory[N_MEM];
float glWeights[N_MEM];
int bufferIndex = 0;

// Precomputed Model Constants (Scaled for TS_SLOW)
float C_e0 = 0.0f;
float C_ehist = 0.0f;
float C_fhist = 0.0f;
float F_slow_memory = 0.0f;                 // Zero-Order Hold register for slow loop forces

// Encoder Configurations
volatile long encoderPos = 0;
volatile int lastEncoded = 0;
const long WALL_POSITION_COUNTS = 400;     
const float COUNTS_TO_DISPLACEMENT = 0.5f; 

PwmOut pwm_pin5(5);
unsigned long lastLoopTime = 0;
int telemetryCounter = 0;

void updateEncoderStandard();

void precomputeConstants() {
  glWeights[0] = 1.0f;
  for (int j = 1; j < N_MEM; j++) {
    glWeights[j] = glWeights[j - 1] * (1.0f - ((ALPHA + 1.0f) / (float)j));
  }
  
  // Base scaling calculation driven entirely by the slow-rate period (TS_SLOW)
  float Ts_alpha = pow(TS_SLOW, ALPHA);
  float tau_alpha = B_1 / K_1;
  float B_dyn = B_1 * (1.0f + (K_0 / K_1));
  float denom = 1.0f + (tau_alpha / Ts_alpha);
  
  C_e0 = (K_0 + (B_dyn / Ts_alpha)) / denom;
  C_ehist = (B_dyn / Ts_alpha) / denom;
  C_fhist = (tau_alpha / Ts_alpha) / denom;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(DIR_PIN, OUTPUT);
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(CURRENT_SEN_PIN, INPUT);

  for(int i = 0; i < N_MEM; i++) {
    errorHistory[i] = 0.0f;
    forceHistory[i] = 0.0f;
  }

  int MSB = digitalRead(CLK_PIN);
  int LSB = digitalRead(DT_PIN);
  lastEncoded = (MSB << 1) | LSB;
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), updateEncoderStandard, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), updateEncoderStandard, CHANGE);

  pwm_pin5.begin(20000.0f, 0.0f); 
  pwm_pin5.pulse_perc(0.0f);

  long zeroSum = 0;
  for(int i = 0; i < 1000; i++) {
    zeroSum += analogRead(CURRENT_SEN_PIN);
    delayMicroseconds(200);
  }
  zero_voltage_baseline = ((float)zeroSum / 1000.0f) * (ADC_VCC / ADC_RESOLUTION);

  precomputeConstants();
  
  lastLoopTime = micros();
  Serial.println(F("# DUAL-RATE FO-SLS ENVIRONMENT INITIALIZED"));
  Serial.println(F("# Columns: Pos,Penetration,F_Th,F_Ren,I_Targ,I_Meas,Alpha"));
}

void loop() {
  if (systemKilled) return;

  // Jitter-Free Deterministic Execution Engine (Strict 1000 Hz Fast Loop Timeline)
  while (micros() - lastLoopTime < 500) { ; }
  lastLoopTime += 500; 

  // 1. Thread-Safe Encoder Sample
  noInterrupts();
  long currentRawPos = encoderPos;
  interrupts();

  // 2. Compute Instantaneous Penetration Error (e_k)
  float e_k = 0.0f;
  if (currentRawPos > WALL_POSITION_COUNTS) {
    e_k = (float)(currentRawPos - WALL_POSITION_COUNTS) * COUNTS_TO_DISPLACEMENT;
  }
  float functionalPenetration = (currentRawPos > WALL_POSITION_COUNTS) ? (float)(currentRawPos - WALL_POSITION_COUNTS) : 0.0f;

  // ==========================================
  // CRITICAL SUB-SYSTEM: DECOUPLED DUAL-RATE ENGINE
  // ==========================================
  slowLoopCounter++;
  if (slowLoopCounter >= N_RATIO) {
    // Execute heavy Grünwald-Letnikov memory summary operations at the slower rate (TS_SLOW)
    float sum_e = 0.0f;
    float sum_f = 0.0f;
    for (int j = 1; j < N_MEM; j++) {
      int fetchIndex = (bufferIndex - j + N_MEM) % N_MEM;
      sum_e += glWeights[j] * errorHistory[fetchIndex];
      sum_f += glWeights[j] * forceHistory[fetchIndex];
    }

    // Compute the slow viscoelastic force contribution
    F_slow_memory = (C_ehist * sum_e) - (C_fhist * sum_f);

    // Commit current values to the ring buffers at the slow clock tick
    errorHistory[bufferIndex] = e_k;
    // Store the baseline slow unconstrained theoretical model force
    forceHistory[bufferIndex] = (C_e0 * e_k) + F_slow_memory; 
    
    bufferIndex = (bufferIndex + 1) % N_MEM;
    slowLoopCounter = 0; // Reset down-sampling counter
  }

  // 3. Fast Loop Synthesis: Instantaneous Spring + Latched Memory Force
  float F_theoretical = (C_e0 * e_k) + F_slow_memory;
  
  float F_rendered = F_theoretical;
  if (F_rendered < 0.0f) F_rendered = 0.0f; 

  // 4. Force to Current Converters
  float targetCurrent = F_rendered * FORCE_TO_CURRENT;
  targetCurrent = constrain(targetCurrent, 0.0f, I_SAFE_LIMIT); 

  // 5. ADC Acquisition and Filtration
  long adcAccumulator = 0;
  for(int s = 0; s < 8; s++) {
    adcAccumulator += analogRead(CURRENT_SEN_PIN);
  }
  float averagedADC = (float)adcAccumulator / 8.0f;
  float voltage = (averagedADC / ADC_RESOLUTION) * ADC_VCC;
  float rawCurrent = (voltage - zero_voltage_baseline) / SENSITIVITY;
  filteredCurrent = (CONFIGURABLE_BETA * rawCurrent) + ((1.0f - CONFIGURABLE_BETA) * filteredCurrent);
  float continuousCurrentMagnitude = abs(filteredCurrent);

  // 6. PI Engine with Conditional Integration Anti-Windup
  float currentError = targetCurrent - continuousCurrentMagnitude;
  float pEffort = Kp_curr * currentError;
  float potentialIntegral = currentIntegralError + (currentError * TS_FAST);
  float iEffort = Ki_curr * potentialIntegral;
  float potentialControlEffort = pEffort + iEffort;

  if ((potentialControlEffort >= 0.0f && potentialControlEffort <= 255.0f) || 
      (potentialControlEffort > 255.0f && currentError < 0.0f) || 
      (potentialControlEffort < 0.0f && currentError > 0.0f)) {
    currentIntegralError = potentialIntegral;
  }
  float controlEffort = pEffort + (Ki_curr * currentIntegralError);

  // 7. PWM Motor Driver Actuation Output
  float dutyCycle = controlEffort / 255.0f;
  dutyCycle = constrain(dutyCycle, 0.0f, 1.0f);
  int rawPwmOut = (int)(dutyCycle * 255.0f);

  if (targetCurrent > 0.005f) {
    bool dirState = LOW; 
    if (INVERT_MOTOR_DIRECTION) dirState = !dirState;
    digitalWrite(DIR_PIN, dirState);
    pwm_pin5.pulse_perc(dutyCycle * 100.0f);
  } else {
    pwm_pin5.pulse_perc(0.0f);
    currentIntegralError = 0.0f; 
  }

  // 8. Python Data Logger Telemetry
  telemetryCounter++;
  if (telemetryCounter >= 25) {
    Serial.print(F("$DAT,"));
    Serial.print(currentRawPos);             Serial.print(F(","));
    Serial.print(functionalPenetration, 1);   Serial.print(F(","));
    Serial.print(F_theoretical, 3);          Serial.print(F(","));
    Serial.print(F_rendered, 3);             Serial.print(F(","));
    Serial.print(targetCurrent, 3);          Serial.print(F(","));
    Serial.print(continuousCurrentMagnitude, 3); Serial.print(F(","));
    Serial.print(ALPHA, 2);
    Serial.println();
    telemetryCounter = 0;
  }
}

void updateEncoderStandard() {
  int MSB = digitalRead(CLK_PIN); 
  int LSB = digitalRead(DT_PIN);
  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderPos++;
  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderPos--;
  lastEncoded = encoded;
}