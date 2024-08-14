#include <WiFi.h>
#include <WiFiClient.h>
#include "LittleFS.h"
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX30100_PulseOximeter.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>


#define TEMPERATURE_SENSOR_PIN 2
#define LDR 36
#define RED 15
#define GREEN 8
#define BLUE 9
#define pressbutton 13
#define REPORTING_PERIOD_MS 1000  // For the heart rate sensor interval
#define ACCEL_REPORTING_PERIOD_MS 4000 // Accelerometer reporting interval
#define TEMP_REPORTING_PERIOD_MS 3000 // Temperature reporting interval
#define LED14  14
#define LED16 16

const char ssid[] = "Hello";
const char pass[] = "chiplips123";
WiFiClient RpClient;
PubSubClient client(RpClient);

OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature sensor(&oneWire);

PulseOximeter pox;

// For the activity level - acceleration
Adafruit_MPU6050 accelerometer;
unsigned long lastActivityTime = 0;  // Variable to store the last time the function was called

uint32_t tsLastReport = 0;

const int mqtt_port = 1883;
const char* mqtt_server = "288feb0bca6a419baa2e5387855973a8.s1.eu.hivemq.cloud";
const char* mqtt_username = "44bb45d3721545f89f6b92ff4afeb6e2.s1.eu.hivemq.cloud";
const char* mqtt_password = "hivemq.webclient.1722650804010";

// Topics
const char* temperature_topic = "pet_tracker/temperature";
const char* activity_topic = "pet_tracker/activity";
const char* heartrate_topic = "pet_tracker/heartrate";
const char* gps_topic = "pet_tracker/gps";
const char* powerstatus = "pet_tracker/powerstatus";

// Adding a serial number for the collar
const char* serial_number = "kelpetNumber1";

float Ax, Ay, Az;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Define the GPS serial port
HardwareSerial gpsSerial(1);


int currentState = LOW;
int lastState = LOW;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
const long ShortPressTime = 1000; // Time for a short press in milliseconds
bool powerOn = false;

unsigned long previousMillis = 0; // Stores last time update
const long interval = 60000; // Update interval (1 minute)

// GPS information
unsigned long lastGPSSendTime = 0;
const unsigned long gpsSendInterval = 600000; // 10 minutes in milliseconds

// For the OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

TaskHandle_t poxReadTaskHId = NULL;
bool poxTaskStarted = false;

const unsigned char wifi_icon_connected [] PROGMEM = {
	0x66, 0xff, 0xe7, 0x7e, 0x3c, 0x18, 0x00, 0x18
};

const unsigned char wifi_icon_disconnected [] PROGMEM = {
	0x00, 0x3c, 0xf7, 0x7e, 0x0c, 0x18, 0x18, 0x00
};

void onBeatDetected() {
  Serial.println("Beat!");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  // Start the GPS serial communication
  gpsSerial.begin(9600, SERIAL_8N1); // RX = GPIO16, TX = GPIO17 (change as per your wiring)

  Serial.println("Starting setup...");
  
  initOLED();
  Serial.println("After initOLED");
  
  WelcomeMessage();
  Serial.println("After WelcomeMessage");
  
   initPulseOximeter();
   Serial.println("After initPulseOximeter");
  
   pox.setOnBeatDetectedCallback(onBeatDetected);
   Serial.println("After setOnBeatDetectedCallback");
  
   stopPox();  // Stop the Pulse Oximeter
   Serial.println("After stopPox");
  
   initAccelerometer();
   Serial.println("After initAccelerometer");
  
   initTemperatureSensor();
   Serial.println("After initTemperatureSensor");
   
   initLEDs();
   Serial.println("After Leds");

  initWiFi(); // Initialize WiFi
  initMQTT(); // Initialize MQTT

  Serial.println("Setup complete.");
}

void loop() {
  Serial.println("In loop");
  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    if (!poxTaskStarted) {
      startReadPoxTask();
      poxTaskStarted = true;
    }
 } 
 else {
    reconnect();
  }
  displayWiFiStatus();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    updateTime();
  }

   //checkHeartRate();
   checkTemperature();
   ActivityLevel();
   GPSData();
  delay(100);  // Adding a small delay to avoid watchdog reset
}

void initPulseOximeter() {
  Serial.println("Initializing pulse oximeter...");
  if (!pox.begin()) {
    Serial.println("FAILED to initialize pulse oximeter");
    while (1);
  } else {
    Serial.println("Pulse oximeter initialized SUCCESSFULLY");
  }
}


void initWiFi(){
    // Begin the Wi-Fi connection
    WiFi.begin(ssid, pass);

    // Print a message to the serial monitor
    Serial.print("Connecting to WiFi");

    // Wait for the connection to be established
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    // Print the IP address once connected
    Serial.println();
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void initMQTT() {
  Serial.println("Initializing MQTT...");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void initOLED() {
  // Initialize the OLED display
   if(!display.begin(SSD1306_SWITCHCAPVCC,SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

// For the oled display
void WelcomeMessage(){
  display.display();
  delay(2000); // Pause for 2 seconds

  display.clearDisplay();
  display.setTextSize(1);
   // Draw a single pixel in white
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,10);
  display.print(F("Welcome to"));
  display.setCursor(0,20);
  display.print(F("Ketlit Health"));
  display.display();
  delay(3000);

  display.clearDisplay();
  display.setCursor(0,10);
  display.print(F("Starting the device..."));
  display.display();
  delay(3000);

  display.clearDisplay();
  display.setCursor(0,10);
  display.print(F("Device Started!"));
  display.display();
  delay(2000);
}

void updateTime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeStr[6];
    char dateStr[7];
    strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "%d %b", &timeinfo);

    display.setCursor(SCREEN_WIDTH - 50, 0); // Adjust cursor position
    display.print(dateStr);
    display.setCursor(SCREEN_WIDTH - 50, 10); // Adjust cursor position
    display.print(timeStr);
    display.display();
  }
}

void displayWiFiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    display.drawBitmap(0, 0, wifi_icon_connected, 16, 16, SSD1306_WHITE);
  } else {
    display.clearDisplay();
    display.drawBitmap(0, 0, wifi_icon_disconnected, 16, 16, SSD1306_WHITE);
  }
}

void initAccelerometer() {
  Serial.println("Initializing accelerometer...");
  if (!accelerometer.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  accelerometer.setAccelerometerRange(MPU6050_RANGE_8_G);
  accelerometer.setGyroRange(MPU6050_RANGE_500_DEG);
  accelerometer.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void initTemperatureSensor() {
  Serial.println("Initializing temperature sensor...");
  sensor.begin();
}

void reconnect() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "Esp32-S2-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker.");
      // Subscribe to necessary topics
      client.subscribe(temperature_topic);  // If needed for receiving data
      client.subscribe(activity_topic);
      client.subscribe(heartrate_topic);
      client.subscribe(powerstatus);
    } else {
      Serial.print("Failed, rc= ");
      Serial.print(client.state());
      Serial.println(". Trying again in 5 seconds.");
      delay(5000);
    }
  }
}

// Starting reading the push
void startReadPox(){
  pox.resume();
}

// Stop the pox
void stopPox(){
  pox.shutdown();
}

/********** MAX30100 TASK *****/
void poxReadTask(void* param) {
  while(1){
    //Make sure to call update as fast as possible
    pox.update();
    vTaskDelay (1 / portTICK_PERIOD_MS);
  }
  poxReadTaskHId = NULL;
  vTaskDelete(NULL); //kill itself
}

void startReadPoxTask() {
  if (poxReadTaskHId == NULL) {
    xTaskCreatePinnedToCore(poxReadTask, "poxReadTask", 10000, NULL, 1, &poxReadTaskHId, 1);
    startReadPox();  // Resume the Pulse Oximeter
  }
}

void checkHeartRate() {  
  float heartrate = pox.getHeartRate();
  float SPO2 = pox.getSpO2();
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      Serial.print("Heart rate:");
      Serial.print(heartrate);
      Serial.print("bpm / SpO2:");
      Serial.print(SPO2);
      Serial.println("%");
      
      tsLastReport = millis();
      publishHeartRateData(heartrate, SPO2);
  }
}

void publishHeartRateData(float heartRate, float SPO2) {
  char message[100];
  snprintf(message, sizeof(message), "{\"serial_number\": \"%s\", \"heart_rate\": %.1f, \"spo2\": %.1f}", serial_number, heartRate, SPO2);
  client.publish(heartrate_topic, message);
}

void updateDisplay(float lat, float lon) {
  // Update OLED display or other UI with current GPS coordinates
  display.clearDisplay();
  display.setCursor(0, 10);
  display.print("Lat: "); display.print(lat);
  display.setCursor(0, 30);
  display.print("Lon: "); display.print(lon);
  display.display();
}

void checkTemperature() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= TEMP_REPORTING_PERIOD_MS) {
    previousMillis = currentMillis;
    
    sensor.requestTemperatures();
    float temperature = sensor.getTempCByIndex(0);

    if (temperature != DEVICE_DISCONNECTED_C) {
      publishTemperatureData(temperature);
      Serial.print("Published Temperature Data: ");
      Serial.println(temperature);
    } else {
      Serial.println("Error reading temperature");
    }
  }
}

void publishTemperatureData(float temperature) {
  // Create payload string including serial number
  char message[100];
  snprintf(message, sizeof(message), "{\"serial_number\": \"%s\", \"temperature\": %.2f}", serial_number, temperature);
  client.publish(temperature_topic, message);
}

void WifiDisplay(){
  display.clearDisplay();
  display.setCursor(0,10);
  display.print(F("WiFi Connected"));
  display.display();
  delay(2000);
}

// For the accelerometer
void ActivityLevel() {
  unsigned long currentMillis = millis();  // Get the current time
  if (currentMillis - lastActivityTime > ACCEL_REPORTING_PERIOD_MS) {
    sensors_event_t a, g, temp;
    accelerometer.getEvent(&a, &g, &temp);
    Ax = a.acceleration.x;
    Ay = a.acceleration.y;
    Az = a.acceleration.z;

    Serial.print("Accelerometer data: ");
    Serial.print(Ax);
    Serial.print(", ");
    Serial.print(Ay);
    Serial.print(", ");
    Serial.println(Az);

    lastActivityTime = millis();
    publishAccelerometerData(Ax, Ay, Az);
  }
}

void publishAccelerometerData(float x, float y, float z) {
  char message[150];
  snprintf(message, sizeof(message), "{\"serial_number\": \"%s\", \"activity\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}}", serial_number, x, y, z);
  client.publish(activity_topic, message);
}


// For the GPS 
void GPSData() {
  unsigned long currentMillis = millis();
  
  // Check if it's time to read the GPS data
  if (currentMillis - lastGPSSendTime >= gpsSendInterval) {
    lastGPSSendTime = currentMillis;

    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
      double latitude = gps.location.lat();
      double longitude = gps.location.lng();
      Serial.print("Latitude: ");
      Serial.println(latitude, 6);
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);

      // Publish GPS data
     publishGPSData(latitude, longitude);
    }
  }
}

void publishGPSData(float latitude, float longitude) {
  char message[150];
  snprintf(message, sizeof(message), "{\"serial_number\": \"%s\", \"latitude\": %.6f, \"longitude\": %.6f}", serial_number, latitude, longitude);
  client.publish(gps_topic, message);
}

void initLEDs() {
  Serial.println("Initializing LEDs...");
  pinMode(LDR, INPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(LED14, OUTPUT);
  pinMode(LED16, OUTPUT);
  pinMode(pressbutton, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // Assuming you're using the built-in LED for power status
  digitalWrite(LED_BUILTIN, LOW); // Initially turn off the power
}

void Press() {
  currentState = digitalRead(pressbutton);

  if (currentState == HIGH && lastState == LOW) {
    pressedTime = millis();
  } else if (currentState == LOW && lastState == HIGH) {
    releasedTime = millis();
    long pressDuration = releasedTime - pressedTime;
    if (pressDuration < ShortPressTime) {
      // Handle short press
      togglePower();
    }
  }
  lastState = currentState;
}

void togglePower() {
  powerOn = !powerOn;
  if (powerOn) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on power
  } else {
    digitalWrite(LED_BUILTIN, LOW); // Turn off power
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);

  if (message == "TOGGLE") {
    togglePower();
  }
}

void setRGBColor(int Red, int Green, int Blue) {
  analogWrite(RED, Red);
  analogWrite(GREEN, Green);
  analogWrite(BLUE, Blue);
}

