/**
 * FO-SLS Zener Model Implementation
 * Corrected based on 2026 Fractional-Order Viscoelasticity Paper
 */

#include "pwm.h" 

// --- HARDWARE PIN DEFINITIONS ---
const int CLK_PIN = 2;
const int DT_PIN = 3;
const int DIR_PIN = 4;
const int MAX_PWM_OUTPUT = 225;

// --- RENDERING GAIN ---
// Start with 10.0f. Increase this until the wall feels solid.
// If it starts buzzing or vibrating, decrease this value.
const float FORCE_GAIN = 5.0f;

// --- SAFETY & HARDWARE CONFIGURATION ---
const bool INVERT_MOTOR_DIRECTION = false; // Restored sign convention toggle
const float MAX_FORCE_OUTPUT = 150.0f;     // Safety: Absolute maximum force allowed
// const float MAX_FORCE_DERIVATIVE = 50.0f;  // Safety: Maximum allowable force change per ms
bool systemKilled = false;                 // Safety: Emergency software kill switch
// float previousForce = 0.0f;                // Safety: For derivative tracking

// --- FO-SLS PARAMETERS (2026 Nomenclature) ---
const float K_0 = 2.89f;    
const float K_1 = 5.70f;   
const float B_1 = 5.89f;   
const float ALPHA = 0.5f;  
const float TS = 0.001f;   // 1ms Sampling Period

// --- DUAL-RING BUFFER CONFIGURATION ---
const int N_MEM = 101;     
float errorHistory[N_MEM];
float forceHistory[N_MEM];
float glWeights[N_MEM];
int bufferIndex = 0;

// --- PRECOMPUTED EXECUTION CONSTANTS ---
float C_e0 = 0.0f;
float C_ehist = 0.0f;
float C_fhist = 0.0f;

// --- ENCODER VARIABLES ---
volatile long encoderPos = 0;
volatile int lastEncoded = 0;
const long WALL_POSITION_COUNTS = 400;

// --- ACTUATION ---
PwmOut pwm_pin5(5);
unsigned long lastLoopTime = 0;
int telemetryCounter = 0;

// --- FUNCTION PROTOTYPES ---
void actuateCytronStable(float forceCommand);
void updateEncoderStandard();
void emergencyShutdown(const char* reason);

void precomputeConstants() {
  glWeights[0] = 1.0f;
  for (int j = 1; j < N_MEM; j++) {
    glWeights[j] = glWeights[j - 1] * (1.0f - ((ALPHA + 1.0f) / (float)j));
  }

  float Ts_alpha = pow(TS, ALPHA);
  float tau_alpha = B_1 / K_1;
  float B_dyn = B_1 * (1.0f + (K_0 / K_1));
  float denom = 1.0f + (tau_alpha / Ts_alpha);

  C_e0 = (K_0 + (B_dyn / Ts_alpha)) / denom;
  C_ehist = (B_dyn / Ts_alpha) / denom;
  C_fhist = (tau_alpha / Ts_alpha) / denom;
}

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  
  // Explicit Parameter Initialization
  for(int i = 0; i < N_MEM; i++) {
    errorHistory[i] = 0.0f;
    forceHistory[i] = 0.0f;
  }
  
  pwm_pin5.begin(20000.0f, 0.0f);
  pwm_pin5.pulse_perc(0.0f); 

  int MSB = digitalRead(CLK_PIN); 
  int LSB = digitalRead(DT_PIN); 
  lastEncoded = (MSB << 1) | LSB;

  attachInterrupt(digitalPinToInterrupt(CLK_PIN), updateEncoderStandard, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), updateEncoderStandard, CHANGE);

  Serial.begin(115200);
  precomputeConstants();
}

void loop() {
  if (systemKilled) return; // Halt loop if safety tripped

  unsigned long currentMicros = micros();
  if (currentMicros - lastLoopTime < 1000) return; 
  lastLoopTime = currentMicros;

  long currentRawPos;
  noInterrupts();
  currentRawPos = encoderPos;
  interrupts();

  float e_k = 0.0f;
  if (currentRawPos > WALL_POSITION_COUNTS) {
    e_k = (float)(currentRawPos - WALL_POSITION_COUNTS);
  }

  float sum_e = 0.0f;
  float sum_f = 0.0f;
  
  for (int j = 1; j < N_MEM; j++) {
    int fetchIndex = (bufferIndex - j + N_MEM) % N_MEM;
    sum_e += glWeights[j] * errorHistory[fetchIndex];
    sum_f += glWeights[j] * forceHistory[fetchIndex];
  }

  float F_k = 0.0f;
  if (currentRawPos > WALL_POSITION_COUNTS) {
    F_k = -( (C_e0 * e_k) + (C_ehist * sum_e) - (C_fhist * sum_f) );
  }

  // MODIFICATION: Clamp F_k BEFORE storing to prevent haptic wind-up
  if (F_k > 0.0f) F_k = 0.0f; 
  
  // SAFETY: Runaway protection limit
  if (F_k > MAX_FORCE_OUTPUT) F_k = MAX_FORCE_OUTPUT;

  // SAFETY: Derivative Limit Check
  // if (abs(F_k - previousForce) > MAX_FORCE_DERIVATIVE) {
  //   emergencyShutdown("ERR: Derivative Limit");
  //   return;
  // }
  // previousForce = F_k;
  float F_theoretical =
    -((C_e0 * e_k) + (C_ehist * sum_e) - (C_fhist * sum_f));

  float F_rendered = F_theoretical;

  if (F_rendered > 0.0f)
      F_rendered = 0.0f;
  // Update buffers with the TRUE rendered force
  errorHistory[bufferIndex] = e_k;
  forceHistory[bufferIndex] = F_theoretical;
  bufferIndex = (bufferIndex + 1) % N_MEM;

  actuateCytronStable(F_rendered);

  // MODIFICATION: Improved Telemetry 
  telemetryCounter++;
  if (telemetryCounter >= 25) { 
    Serial.print(millis()); Serial.print(",");
    Serial.print(currentRawPos); Serial.print(",");
    Serial.print(e_k); Serial.print(",");
    Serial.print(F_k); Serial.print(",");
    Serial.println((int)abs(F_k)); // PWM equivalent
    telemetryCounter = 0;
  }
}

void actuateCytronStable(float forceCommand) {
    // Multiply the calculated force by the gain to map to PWM units
    int pwmValue = abs((int)(forceCommand * FORCE_GAIN)); 
    
    if (pwmValue > MAX_PWM_OUTPUT) pwmValue = MAX_PWM_OUTPUT;
    if (pwmValue < 0) pwmValue = 0;

    float dutyPercentage = (float)pwmValue * (100.0f / 255.0f);

    bool directionState = (forceCommand > 0);
    if (INVERT_MOTOR_DIRECTION) directionState = !directionState;

    digitalWrite(DIR_PIN, directionState ? HIGH : LOW);
    pwm_pin5.pulse_perc(dutyPercentage);
}

void emergencyShutdown(const char* reason) {
  systemKilled = true;
  pwm_pin5.pulse_perc(0.0f);
  digitalWrite(DIR_PIN, LOW);
  Serial.println(reason);
}

void updateEncoderStandard() {
  int MSB = digitalRead(CLK_PIN); int LSB = digitalRead(DT_PIN); 
  int encoded = (MSB << 1) | LSB; int sum = (lastEncoded << 2) | encoded; 
  if(sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderPos++;
  if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderPos--;
  lastEncoded = encoded; 
}