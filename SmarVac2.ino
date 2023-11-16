#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <LiquidCrystal_I2C.h>
#include "ThingSpeak.h"
#include "WiFiS3.h"

// LCD and sensors initialization
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
const char* WIFI_NAME = "iPhone (2)";
const char* WIFI_PASSWORD = "HLIA-4mfw-UtVU-g8AT";
const int myChannelNumber = 2298977; //thingspeak channel ID
const char* myApiKey = "Q9M0PZL6F34SD7C0"; //write API key from ThingSpeak
const char* server = "api.thingspeak.com";


WiFiClient client;

// Define pin numbers
const int buttonPin = 7; // Button pin for reset
const int ledPinGreen = 8; // Green LED pin
const int ledPinRed = 9; // Red LED pin
const int dsMeasurePin = A0; // Dust sensor measurement pin
const int dsLed = 4; // Dust sensor LED pin

// Variables to hold sensor readings
float baselinePressure;
float baselineDustDensity;

// Thresholds
const float pressureThreshold = 5; // Threshold for pressure change
const float dustDensityThreshold = 0.5; // Threshold for dust density change
const unsigned long timeThreshold = 180000; // Time in milliseconds to trigger an alert

void setup(void) {
  Serial.begin(9600);
  Serial.println("Air Filter Monitor Starting");


  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Filter Start");

  delay(1000);

  pinMode(dsLed, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Use the internal pull-up resistor
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinRed, OUTPUT);

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Wifi not connected");
  }
  Serial.println("Wifi connected !");
  Serial.println("Local IP: " + String(WiFi.localIP()));
  ThingSpeak.begin(client);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Wifi OK");
  delay(1000);
  
  // Initialize the BMP085 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    lcd.print("Sensor error");
    while (1);
  }

  // Get baseline sensor values
  resetBaseline();
}

void loop(void) {
  if (digitalRead(buttonPin) == LOW) { // Button pressed
    delay(200); // Debounce delay
    if (digitalRead(buttonPin) == LOW) { // Check again if still pressed
      resetBaseline();
    }
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Reading");
  delay(5000);
  // Read current sensor values
  float currentPressure = getPressure();
  float currentDustDensity = getDustDensity();
  ThingSpeak.setField(1,currentPressure);
  ThingSpeak.setField(2,currentDustDensity);

  int x = ThingSpeak.writeFields(myChannelNumber,myApiKey);

  if(x == 200){
    Serial.println("Data pushed successfull");
  }else{
    Serial.println("Push error" + String(x));
  }
  // Check for condition to change the filter based on pressure or dust density
  if (millis() - startTime >= timeThreshold ||
      abs(currentPressure - baselinePressure) >= pressureThreshold || 
      abs(currentDustDensity - baselineDustDensity) >= dustDensityThreshold) {
    notifyUser();
  } else {
    // No alert condition, green LED is on and red LED is off
    digitalWrite(ledPinGreen, HIGH);
    digitalWrite(ledPinRed, LOW);
  }

  // Display current sensor values on LCD
  displayReadings(currentPressure, currentDustDensity);
  
  // Wait for some time before the next reading
  delay(20000); // Delay for 20 seconds
}

void resetBaseline() {
  baselinePressure = getPressure();
  baselineDustDensity = getDustDensity();
  startTime = millis();
  digitalWrite(ledPinGreen, HIGH);
  digitalWrite(ledPinRed, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Baseline Reset");
}

float getPressure() {
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    return event.pressure;
  } else {
    Serial.println("Sensor error");
    return 0; // Return 0 if there's a sensor error
  }
}

float getDustDensity() {
  digitalWrite(dsLed, LOW);
  delay(280); // According to the sensor's datasheet
  float voMeasured = analogRead(dsMeasurePin);
  digitalWrite(dsLed, HIGH);

  float calcVoltage = voMeasured * (5.0 / 1024.0);
  float dustDensity = 0.17 * calcVoltage - 0.1;
  return dustDensity;
}

void notifyUser() {
  digitalWrite(ledPinRed, HIGH);
  digitalWrite(ledPinGreen, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CHANGE FILTER!");

  // Wait here until the button is pressed to acknowledge the alert
  while (digitalRead(buttonPin) == HIGH) {
    // Button is not pressed, so wait
    delay(50); // Small delay to debounce and prevent excessive reading
  }

  // Once the button is pressed (and released), we can reset the baseline
  while (digitalRead(buttonPin) == LOW) {
    // Wait for the button to be released
    delay(50); // Small delay to debounce and prevent excessive reading
  }

  resetBaseline(); // Now we reset the baseline readings
}

void displayReadings(float pressure, float dustDensity) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure: ");
  lcd.print(pressure);
  lcd.setCursor(0, 1);
  lcd.print("Dust: ");
  lcd.print(dustDensity);
}