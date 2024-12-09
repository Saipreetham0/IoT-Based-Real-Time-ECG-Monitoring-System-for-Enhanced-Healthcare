#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h>

#define WIFISSID "KSP"                              // Enter WifiSSID here
#define PASSWORD "9550421866"                       // Enter password here
#define TOKEN "BBUS-8aiVRPtbvrakumJenefspzi4tEXDpc" // Ubidots' TOKEN
#define MQTT_CLIENT_NAME "mymqttclient"             // MQTT client Name

// Ubidots Variable Labels
#define VARIABLE_LABEL "ecg-sensor"
#define VARIABLE_LABEL_TEMP "temp"
#define VARIABLE_LABEL_SPO2 "spo2"
#define VARIABLE_LABEL_HEART_RATE "heart_rate"
#define DEVICE_LABEL "healthmonitor" // ubidots device label

#define SENSORPIN A0 // Set the A0 as SENSORPIN
char mqttBroker[] = "industrial.api.ubidots.com";
char payload[10000];
char topic[150];
// Space to store values to send
char str_sensor[10];
char str_millis[20];
double epochseconds = 0;
double epochmilliseconds = 0;
double current_millis = 0;
double current_millis_at_sensordata = 0;
double timestampp = 0;
int j = 0;
/****************************************
   Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
void callback(char *topic, byte *payload, unsigned int length)
{
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    Serial.write(payload, length);
    Serial.println(topic);
}
void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.println("Attempting MQTT connection...");
        // Attemp to connect
        if (client.connect(MQTT_CLIENT_NAME, TOKEN, ""))
        {
            Serial.println("Connected");
        }
        else
        {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2 seconds");
            // Wait 2 seconds before retrying
            delay(2000);
        }
    }
}

// Pin for DS18B20
#define ONE_WIRE_BUS 4 // D4 pin of ESP32

// Create an instance of the OneWire and DallasTemperature libraries
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Create LCD object with I2C address (usually 0x27 or 0x3F) and dimensions
LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100];  // Infrared LED sensor data
uint32_t redBuffer[100]; // Red LED sensor data
int32_t bufferLength;    // Data length
int32_t spo2;            // SpO2 value
int8_t validSPO2;        // Indicator for valid SpO2 calculation
int32_t heartRate;       // Heart rate value
int8_t validHeartRate;   // Indicator for valid heart rate calculation

// Function to publish data for a specific variable
void publishVariableData(const char *variableLabel, float value)
{
    char localPayload[300];
    char strValue[10];
    char strTimestamp[20];

    sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

    // Convert value to string
    dtostrf(value, 4, 2, strValue);

    // Calculate timestamp
    current_millis_at_sensordata = millis();
    timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
    dtostrf(timestampp, 10, 0, strTimestamp);

    // Create payload
    sprintf(localPayload, "{\"%s\": [{\"value\": %s, \"timestamp\": %s}]}",
            variableLabel, strValue, strTimestamp);

    // Publish payload
    client.publish(topic, localPayload);

    // Print for debugging
    Serial.print("Publishing ");
    Serial.print(variableLabel);
    Serial.print(": ");
    Serial.println(localPayload);
}

void setup()
{

    Serial.begin(115200);
    WiFi.begin(WIFISSID, PASSWORD);
    // Assign the pin as INPUT
    pinMode(SENSORPIN, INPUT);
    Serial.println();
    Serial.print("Waiting for WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("");
    Serial.println("WiFi Connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    timeClient.begin();
    client.setServer(mqttBroker, 1883);
    client.setCallback(callback);
    timeClient.update();
    epochseconds = timeClient.getEpochTime();
    epochmilliseconds = epochseconds * 1000;
    Serial.print("epochmilliseconds=");
    Serial.println(epochmilliseconds);
    current_millis = millis();
    Serial.print("current_millis=");
    Serial.println(current_millis);

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    delay(2000);

    // Initialize MAX30105 sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        lcd.setCursor(0, 0);
        lcd.print("Sensor Error");
        Serial.println("MAX30105 Sensor Initialization Failed!");
        while (1)
            ; // Halt if sensor initialization fails
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Attach Sensor");

    // Initialize DS18B20 temperature sensor
    sensors.begin();

    // MAX30105 configuration
    byte ledBrightness = 100; // LED brightness (0=Off, 255=50mA)
    byte sampleAverage = 4;   // Average samples
    byte ledMode = 2;         // Red + IR
    byte sampleRate = 100;    // Sampling rate in Hz
    int pulseWidth = 411;     // Pulse width
    int adcRange = 4096;      // ADC range

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    delay(2000); // Allow time for initialization
}

void loop()
{

    if (!client.connected())
    {
        reconnect();
        j = 0;
    }
    // sprintf(payload, "%s", "{\"ECG_Sensor_data\": [{\"value\":1234, \"timestamp\": 1595972075},{\"value\":1111, \"timestamp\": 1595971075},{\"value\":2222, \"timestamp\": 1595970075}]}");
    j = j + 1;
    Serial.print("j=");
    Serial.println(j);
    sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");                     // Cleans the payload
    sprintf(payload, "{\"%s\": [", VARIABLE_LABEL); // Adds the variable label
    for (int i = 1; i <= 3; i++)
    {
        float sensor = analogRead(SENSORPIN);
        dtostrf(sensor, 4, 2, str_sensor);
        sprintf(payload, "%s{\"value\":", payload);      // Adds the value
        sprintf(payload, "%s %s,", payload, str_sensor); // Adds the value
        current_millis_at_sensordata = millis();
        timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
        dtostrf(timestampp, 10, 0, str_millis);
        sprintf(payload, "%s \"timestamp\": %s},", payload, str_millis); // Adds the value
        delay(150);
    }
    float sensor = analogRead(SENSORPIN);
    dtostrf(sensor, 4, 2, str_sensor);
    current_millis_at_sensordata = millis();
    timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
    dtostrf(timestampp, 10, 0, str_millis);
    sprintf(payload, "%s{\"value\":%s, \"timestamp\": %s}]}", payload, str_sensor, str_millis);
    Serial.println("Publishing data to Ubidots Cloud");
    client.publish(topic, payload);
    Serial.println(payload);
    // client.loop();

    bufferLength = 100; // Store 4 seconds of samples at 25 samples per second

    // Read first 100 samples
    for (byte i = 0; i < bufferLength; i++)
    {
        while (!particleSensor.available())
        {
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
    if (validHeartRate && heartRate > 30 && heartRate < 300)
    {
        lcd.print(heartRate); // Print HR
        lcd.print(" ");
        publishVariableData(VARIABLE_LABEL_HEART_RATE, heartRate);
    }
    else
    {
        lcd.print("-- ");
    }

    lcd.print("SpO2:");
    if (validSPO2 && spo2 > 70 && spo2 <= 100)
    {
        lcd.print(spo2); // Print SpO2
        lcd.print("%");
        publishVariableData(VARIABLE_LABEL_SPO2, spo2);
    }
    else
    {
        lcd.print("--%");
    }

    lcd.setCursor(0, 1);
    lcd.print("Temp:");
    if (temperature != DEVICE_DISCONNECTED_C)
    {
        lcd.print(temperature, 2); // Print temperature to 2 decimal places
        lcd.print("C");
        publishVariableData(VARIABLE_LABEL_TEMP, temperature);
    }
    else
    {
        lcd.print("Error");
    }

    // Print data to Serial Monitor
    Serial.print("Heart Rate: ");
    if (validHeartRate && heartRate > 30 && heartRate < 300)
    {
        Serial.print(heartRate);
    }
    else
    {
        Serial.print("--");
    }

    Serial.print(" bpm | SpO2: ");
    if (validSPO2 && spo2 > 70 && spo2 <= 100)
    {
        Serial.print(spo2);
        Serial.print("%");
    }
    else
    {
        Serial.print("--%");
    }

    Serial.print(" | Temperature: ");
    if (temperature != DEVICE_DISCONNECTED_C)
    {
        Serial.print(temperature, 2);
        Serial.println(" °C");
    }
    else
    {
        Serial.println("Error");
    }

    // Refresh readings
    for (byte i = 25; i < 100; i++)
    {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
    }

    for (byte i = 75; i < 100; i++)
    {
        while (!particleSensor.available())
        {
            particleSensor.check();
        }

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    delay(1000); // Delay for readability
}
