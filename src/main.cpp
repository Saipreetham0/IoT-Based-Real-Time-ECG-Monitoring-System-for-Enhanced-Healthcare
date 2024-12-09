#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin for DS18B20
#define ONE_WIRE_BUS 4 // D4 pin of ESP32

// Create an instance of the OneWire and DallasTemperature libraries
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Create LCD object with I2C address (usually 0x27 or 0x3F) and dimensions
LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100]; // Infrared LED sensor data
uint32_t redBuffer[100]; // Red LED sensor data
int32_t bufferLength; // Data length
int32_t spo2; // SpO2 value
int8_t validSPO2; // Indicator for valid SpO2 calculation
int32_t heartRate; // Heart rate value
int8_t validHeartRate; // Indicator for valid heart rate calculation

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(2000);

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error");
    Serial.println("MAX30105 Sensor Initialization Failed!");
    while (1); // Halt if sensor initialization fails
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Attach Sensor");

  // Initialize DS18B20 temperature sensor
  sensors.begin();

  // MAX30105 configuration
  byte ledBrightness = 60; // LED brightness (0=Off, 255=50mA)
  byte sampleAverage = 4;  // Average samples
  byte ledMode = 2;        // Red + IR
  byte sampleRate = 100;   // Sampling rate in Hz
  int pulseWidth = 411;    // Pulse width
  int adcRange = 4096;     // ADC range

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  delay(2000); // Allow time for initialization
}

void loop() {
  bufferLength = 100; // Store 4 seconds of samples at 25 samples per second

  // Read first 100 samples
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Read temperature from DS18B20
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0); // Get the first sensor's temperature

  // Display data on LCD
  lcd.setCursor(0, 0);
  lcd.print("HR:");
  if (validHeartRate && heartRate > 30 && heartRate < 300) {
    lcd.print(heartRate); // Print HR
    lcd.print(" ");
  } else {
    lcd.print("-- ");
  }

  lcd.print("SpO2:");
  if (validSPO2 && spo2 > 70 && spo2 <= 100) {
    lcd.print(spo2); // Print SpO2
    lcd.print("%");
  } else {
    lcd.print("--%");
  }

  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  if (temperature != DEVICE_DISCONNECTED_C) {
    lcd.print(temperature, 2); // Print temperature to 2 decimal places
    lcd.print("C");
  } else {
    lcd.print("Error");
  }

  // Print data to Serial Monitor
  Serial.print("Heart Rate: ");
  if (validHeartRate && heartRate > 30 && heartRate < 300) {
    Serial.print(heartRate);
  } else {
    Serial.print("--");
  }

  Serial.print(" bpm | SpO2: ");
  if (validSPO2 && spo2 > 70 && spo2 <= 100) {
    Serial.print(spo2);
    Serial.print("%");
  } else {
    Serial.print("--%");
  }

  Serial.print(" | Temperature: ");
  if (temperature != DEVICE_DISCONNECTED_C) {
    Serial.print(temperature, 2);
    Serial.println(" °C");
  } else {
    Serial.println("Error");
  }

  // Refresh readings
  for (byte i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  for (byte i = 75; i < 100; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  delay(1000); // Delay for readability
}
