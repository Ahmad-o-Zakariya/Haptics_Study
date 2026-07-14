#include "MAX6675.h" // Rob Tillaart's specific library header
#include <OneWire.h>
#include <DallasTemperature.h>

// --- Pin Map ---
const int thermoSO  = 12; // MISO Line
const int thermoSCK = 13; // Clock Line
const int thermoCS1 = 10; // Chip Select 1 (Top Wall)
const int thermoCS2 = 9;  // Chip Select 2 (Bottom Wall)

const int MQ7_exhaust = A0; 
const int MQ7_ambient = A1; 

const int ONE_WIRE_BUS = 2; 

// --- Objects ---
// NOTE: Rob Tillaart's constructor order is strict: (Select/CS, MISO/SO, Clock/SCK)
MAX6675 wallTemp1(thermoCS1, thermoSO, thermoSCK);
MAX6675 wallTemp2(thermoCS2, thermoSO, thermoSCK);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature roomSensors(&oneWire);

unsigned long startTime;

void setup() {
  Serial.begin(9600);
  
  // Initialize the room sensor array
  roomSensors.begin();
  
  // CRITICAL FOR ROB TILLAART'S LIBRARY: Initialize both thermocouples
  wallTemp1.begin();
  wallTemp2.begin();
  
  startTime = millis();

  // CSV Column Headers for Excel tracking
  Serial.println("Elapsed_Seconds,Wall_Side_C,Wall_Top_C,Gas_Ambient_Raw_window,Gas_Ambient_Raw_table,Room_Temp_Floor_C,Room_Temp_2_C");
}

void loop() {
  unsigned long elapsedSeconds = (millis() - startTime) / 1000;

  // 1. Read K-Type Thermocouples (Stove Wall)
  // Rob Tillaart's library requires you to trigger .read() before requesting the data
  wallTemp1.read();
  float wTop = wallTemp1.getCelsius();
  delay(250); 
  
  wallTemp2.read();
  float wBot = wallTemp2.getCelsius();
  delay(250);

  // 2. Read MQ-7 Carbon Monoxide Sensors
  int gasExhaust = analogRead(MQ7_exhaust);
  int gasAmbient = analogRead(MQ7_ambient);

  // 3. Read DS18B20 Room Array
  roomSensors.requestTemperatures();
  float rTemp1 = roomSensors.getTempCByIndex(0); 
  float rTemp2 = roomSensors.getTempCByIndex(1); 

  // 4. Stream Data in CSV Format
  Serial.print(elapsedSeconds);  Serial.print(",");
  Serial.print(wTop);            Serial.print(",");
  Serial.print(wBot);            Serial.print(",");
  Serial.print(gasExhaust);      Serial.print(",");
  Serial.print(gasAmbient);      Serial.print(",");
  Serial.print(rTemp1);          Serial.print(",");
  Serial.println(rTemp2);        

  // Data log timing adjustment (5000ms total loop window)
  delay(4500); 
}