// --- Tailored for Orange 400 PPR Encoder (4x Mode = 1600 CPR) ---
const float COUNTS_PER_REV = 1600.0; 

// Hardware Pins
const int clkPin = 2; 
const int dtPin = 3;  

volatile long encoderPos = 0;
volatile int lastEncoded = 0;

// High-Speed Timing Constraints (fs = 1000Hz -> Interval = 1000 microseconds)
unsigned long lastTimeMicros = 0;
float lastAngularPosition = 0.0;
const unsigned long SAMPLE_INTERVAL_MICROS = 1000; 

// --- Discrete Butterworth LPF Coefficients (fc = 5Hz, fs = 1000Hz) ---
const float b0 = 0.000945;
const float b1 = 0.001889;
const float b2 = 0.000945;
const float a1 = -1.911197;
const float a2 = 0.914976;

// Filter Time Histories
float x_hist[2] = {0.0, 0.0}; 
float y_hist[2] = {0.0, 0.0}; 

void setup() {
  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dtPin, INPUT_PULLUP);

  int MSB = digitalRead(clkPin); 
  int LSB = digitalRead(dtPin);  
  lastEncoded = (MSB << 1) | LSB; 

  attachInterrupt(digitalPinToInterrupt(clkPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dtPin), updateEncoder, CHANGE);

  // Initialize random seed using floating analog voltage
  randomSeed(analogRead(0));

  Serial.begin(115200);
}

void loop() {
  unsigned long currentTimeMicros = micros();
  
  // High-precision microsecond execution block for true 1kHz pacing
  if (currentTimeMicros - lastTimeMicros >= SAMPLE_INTERVAL_MICROS) {
    long currentRawPos;

    noInterrupts();
    currentRawPos = encoderPos;
    interrupts();

    // ==========================================================
    // STEP 1: GENERATE & SUPERIMPOSE POSITION NOISE
    // Simulates +/- 8 raw counts of high-frequency white noise
    // ==========================================================
    long positionNoise = random(-8, 9); 
    long noisyRawPos = currentRawPos + positionNoise;

    // STEP 2: RUN THE REST OF THE PROCESSING PIPELINE
    // Degrade the noisy signal down to 100 CPR (1/16th scale)
    long degradedSteps = noisyRawPos / 16; 
    
    float totalDegrees = (degradedSteps * 360.0) / 100.0;
    float angularPosition = fmod(totalDegrees, 360.0);
    if (angularPosition < 0) {
      angularPosition += 360.0; 
    }

    // Compute Instantaneous Velocity (dt is now exactly 0.001 seconds)
    float dt = (currentTimeMicros - lastTimeMicros) / 1000000.0;
    float deltaAngle = angularPosition - lastAngularPosition;
    
    if (deltaAngle > 180.0)       deltaAngle -= 360.0;
    else if (deltaAngle < -180.0) deltaAngle += 360.0;

    // Because of the noise + 1000Hz derivative math, this raw calculation is chaotic
    float rawNoisyVelocity = deltaAngle / dt; 

    // STEP 3: EXECUTE THE 1kHz DIGITAL BUTTERWORTH FILTER
    float filteredVelocity = (b0 * rawNoisyVelocity) + (b1 * x_hist[0]) + (b2 * x_hist[1]) 
                             - (a1 * y_hist[0]) - (a2 * y_hist[1]);

    // Update historical delay tracking
    x_hist[1] = x_hist[0];     
    x_hist[0] = rawNoisyVelocity;   
    y_hist[1] = y_hist[0];     
    y_hist[0] = filteredVelocity; 

    // STEP 4: STREAM OUT TELEMETRY
    Serial.print(degradedSteps);
    Serial.print(",");
    Serial.print(angularPosition, 2);
    Serial.print(",");
    Serial.println(rawNoisyVelocity, 2);

    lastAngularPosition = angularPosition;
    lastTimeMicros = currentTimeMicros;
  }
}

void updateEncoder() {
  int MSB = digitalRead(clkPin); 
  int LSB = digitalRead(dtPin);  

  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded; 

  if(sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderPos++;
  if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderPos--;

  lastEncoded = encoded; 
}