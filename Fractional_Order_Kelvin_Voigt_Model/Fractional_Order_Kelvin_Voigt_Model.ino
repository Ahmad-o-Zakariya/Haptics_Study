/**

 * CONFIG 3: 1-DOF Fractional-Order Virtual Wall

 * Implements Grünwald-Letnikov Finite-Memory Realization

 */



#include "pwm.h"  

const float MAX_SAFE_DERIVATIVE = 800.0f; // Lower this if it still spins too fast
bool systemTripped = false;

const int COUNTS_PER_REV = 1600;

const int clkPin = 2;

const int dtPin = 3;

const int dirPin = 4;



volatile long encoderPos = 0;

volatile int lastEncoded = 0;



unsigned long lastTimeMicros = 0;

long lastRawPosition = 0; 

const unsigned long SAMPLE_INTERVAL_MICROS = 1000; 

int telemetryCounter = 0;



// --- FRACTIONAL ORDER SETUP ---

const float MU = 1.5f;       // CHANGE THIS: 0.5, 0.8, 1.0, 1.2, 1.5

const int MEM_LENGTH = 100;  // L: Finite memory window

float gl_weights[MEM_LENGTH];

float x_ring_buffer[MEM_LENGTH];

int buffer_idx = 0;

float Ts = 0.001f;           // 1ms sample time

float Ts_pow_mu = 0.0f;      // Will store Ts^mu



// Haptic Setup 

const long WALL_POSITION_COUNTS = 400;  

const float K_STIFFNESS = 8.0f;       

const float B_FRACTIONAL = 0.01f; // WARNING: Tune carefully. Units change with mu!

const int MAX_PWM_OUTPUT = 225;         

const bool INVERT_MOTOR_DIRECTION = false; 



PwmOut pwm_pin5(5);



void setup() {

  pinMode(dirPin, OUTPUT); 

  digitalWrite(dirPin, LOW);

  pinMode(clkPin, INPUT_PULLUP);

  pinMode(dtPin, INPUT_PULLUP);



  pwm_pin5.begin(20000.0f, 0.0f);

  pwm_pin5.pulse_perc(0.0f); 



  int MSB = digitalRead(clkPin); 

  int LSB = digitalRead(dtPin); 

  lastEncoded = (MSB << 1) | LSB;



  attachInterrupt(digitalPinToInterrupt(clkPin), updateEncoderStandard, CHANGE);

  attachInterrupt(digitalPinToInterrupt(dtPin), updateEncoderStandard, CHANGE);



  Serial.begin(115200);



  // --- PRECOMPUTE GL WEIGHTS ---

  gl_weights[0] = 1.0f;

  for (int j = 1; j < MEM_LENGTH; j++) {

    gl_weights[j] = gl_weights[j - 1] * (1.0f - ((MU + 1.0f) / (float)j));

  }

  Ts_pow_mu = pow(Ts, MU);

  

  // Clear ring buffer

  for(int i=0; i<MEM_LENGTH; i++) x_ring_buffer[i] = 0.0f;



  lastTimeMicros = micros();

}



void loop() {

  unsigned long currentTimeMicros = micros();

  if (currentTimeMicros - lastTimeMicros >= SAMPLE_INTERVAL_MICROS) {

    long currentRawPos;

    noInterrupts();

    currentRawPos = encoderPos;

    interrupts();



    float wallForce = 0.0f;

    float currentPenetration = 0.0f;



    // Calculate penetration depth

    if (currentRawPos > WALL_POSITION_COUNTS) {

      currentPenetration = (float)(currentRawPos - WALL_POSITION_COUNTS);

    } else {

      currentPenetration = 0.0f; // Outside wall, zero penetration

    }



    // Update Ring Buffer

    x_ring_buffer[buffer_idx] = currentPenetration;



    // Calculate GL Fractional Derivative

    float gl_sum = 0.0f;

    for (int j = 0; j < MEM_LENGTH; j++) {

      int read_idx = buffer_idx - j;

      if (read_idx < 0) read_idx += MEM_LENGTH; // Wrap around

      gl_sum += gl_weights[j] * x_ring_buffer[read_idx];

    }

    float fractionalDerivative = gl_sum / Ts_pow_mu;
//     if (abs(fractionalDerivative) > MAX_SAFE_DERIVATIVE || systemTripped) {
//   systemTripped = true;
//   pwm_pin5.pulse_perc(0.0f); // FORCE MOTOR OFF
//   digitalWrite(dirPin, LOW);
  
//   static unsigned long lastErrorPrint = 0;
//   if (millis() - lastErrorPrint > 1000) {
//     Serial.println("!!! EMERGENCY SHUTDOWN: RUNAWAY VELOCITY DETECTED !!!");
//     lastErrorPrint = millis();
//   }
//   return; // Skip the rest of the loop, preventing actuation
// }


    // Advance buffer index

    buffer_idx = (buffer_idx + 1) % MEM_LENGTH;



    // Compute Forces (Only if inside wall)

    if (currentRawPos > WALL_POSITION_COUNTS) {

      float springForce = K_STIFFNESS * currentPenetration;

      float fractionalDampingForce = B_FRACTIONAL * fractionalDerivative;

      

      wallForce = -(springForce + fractionalDampingForce);

      if (wallForce > 0.0f) wallForce = 0.0f; // Unilateral wall constraint

    }



    actuateCytronStable(wallForce);



    // Telemetry

    telemetryCounter++;

    if (telemetryCounter >= 25) { // 40Hz update rate

      Serial.print(currentRawPos); Serial.print(",");

      Serial.print(fractionalDerivative, 2); Serial.print(",");

      Serial.println(wallForce, 1);

      telemetryCounter = 0;

    }

    lastTimeMicros = currentTimeMicros;

  }

}



void actuateCytronStable(float forceCommand) {

  int pwmValue = abs((int)forceCommand);

  if (pwmValue > MAX_PWM_OUTPUT) pwmValue = MAX_PWM_OUTPUT;

  float dutyPercentage = (float)pwmValue * (100.0f / 255.0f);

  bool directionState = (forceCommand > 0);

  if (INVERT_MOTOR_DIRECTION) directionState = !directionState;

  digitalWrite(dirPin, directionState ? HIGH : LOW);

  pwm_pin5.pulse_perc(dutyPercentage); 

}



void updateEncoderStandard() {

  int MSB = digitalRead(clkPin); int LSB = digitalRead(dtPin); 

  int encoded = (MSB << 1) | LSB; int sum = (lastEncoded << 2) | encoded; 

  if(sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderPos++;

  if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderPos--;

  lastEncoded = encoded; 

}
