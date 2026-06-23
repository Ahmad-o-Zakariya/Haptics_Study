/**
 * CONFIG 2: 1-DOF Haptic Virtual Wall - Cytron MD10C R3 Stable Edition
 * High-Frequency 20kHz Actuation using Standard Framework Core API
 * Target Pins: PWM = Pin 5 (PwmOut API), DIR = Pin 4 (digitalWrite)
 */

#include "pwm.h"  

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

// 2nd-Order Butterworth Filter
const float b0 = 0.067455, b1 = 0.134911, b2 = 0.067455;
const float a1 = -1.142981, a2 = 0.412802;
float x_hist[2] = {0.0, 0.0}, y_hist[2] = {0.0, 0.0};

// Haptic Setup 
const long WALL_POSITION_COUNTS = 400;  
const float K_STIFFNESS = 5.0f; // Lowered to protect hardware from shocks       
const float B_DAMPING = 0.25f;           
const int MAX_PWM_OUTPUT = 255;         
const bool INVERT_MOTOR_DIRECTION = true; 

PwmOut pwm_pin5(5);

void setup() {
  pinMode(dirPin, OUTPUT); 
  digitalWrite(dirPin, LOW);
  
  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dtPin, INPUT_PULLUP);

  // Initialize Renesas GPT to 20,000 Hz
  pwm_pin5.begin(20000.0f, 0.0f);
  pwm_pin5.pulse_perc(0.0f); 

  int MSB = digitalRead(clkPin); 
  int LSB = digitalRead(dtPin); 
  lastEncoded = (MSB << 1) | LSB;

  attachInterrupt(digitalPinToInterrupt(clkPin), updateEncoderStandard, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dtPin), updateEncoderStandard, CHANGE);

  Serial.begin(115200);
  lastTimeMicros = micros();
}

void loop() {
  unsigned long currentTimeMicros = micros();
  if (currentTimeMicros - lastTimeMicros >= SAMPLE_INTERVAL_MICROS) {
    long currentRawPos;
    noInterrupts();
    currentRawPos = encoderPos;
    interrupts();

    float dt = 0.001f; 
    long deltaCounts = currentRawPos - lastRawPosition; 
    float rawVelocityCounts = (float)deltaCounts / dt;

    float filteredVelocityCounts = (b0 * rawVelocityCounts) + (b1 * x_hist[0]) + (b2 * x_hist[1]) 
                                 - (a1 * y_hist[0]) - (a2 * y_hist[1]);
    x_hist[1] = x_hist[0]; x_hist[0] = rawVelocityCounts;   
    y_hist[1] = y_hist[0]; y_hist[0] = filteredVelocityCounts;

    float wallForce = 0.0f;
    if (currentRawPos > WALL_POSITION_COUNTS) {
      long penetrationDepthCounts = currentRawPos - WALL_POSITION_COUNTS;
      float springForce = K_STIFFNESS * (float)penetrationDepthCounts;
      float dampingForce = B_DAMPING * filteredVelocityCounts;
      wallForce = -(springForce + dampingForce);
      if (wallForce > 0.0f) wallForce = 0.0f; 
    }

    actuateCytronStable(wallForce);

    telemetryCounter++;
    if (telemetryCounter >= 25) {
      int activePwm = abs((int)wallForce);
      if (activePwm > MAX_PWM_OUTPUT) activePwm = MAX_PWM_OUTPUT;
      Serial.print("CountPos:");   Serial.print(currentRawPos);
      Serial.print(",ForceCmd:");  Serial.print(wallForce, 1);
      Serial.print(",PWM_Out:");   Serial.println(activePwm); 
      telemetryCounter = 0;
    }
    lastRawPosition = currentRawPos;
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
  int MSB = digitalRead(clkPin); 
  int LSB = digitalRead(dtPin); 
  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded; 
  if(sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderPos++;
  if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderPos--;
  lastEncoded = encoded; 
}