#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <LittleFS_Mbed_RP2040.h>
#include "html.h" 
#include <Arduino_LSM6DSOX.h>
#include <PDM.h> 
#include <PubSubClient.h>
//#include <SoftwareSerial.h>
#include <Adafruit_GFX.h> 
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30100_PulseOximeter.h"
#include <TinyGPS++.h>

// Pin definitions
#define pressbutton 7
#define temperaturesensor 2
#define LDR 1 // A1
#define Battery 2 // A0
#define Pin_Red 4 //GPIO23
#define Pin_Green 5
#define Pin_Blue 6
#define RxPin 4
#define TxPin 3
#define lightstrip 33
#define WIDTH 128
#define HEIGHT 64
#define OLED_RESET -1
#define DEBOUNCE_TIME 50

// Variables and constants
PulseOximeter pox;
#define REPORTING_PERIOD 10000
uint32_t LastTimeHeartBeat = 0;

//SoftwareSerial GPS_SERIAL(RxPin, TxPin);
TinyGPSPlus gps;
#define GPS_BAUDRATE 9600

OneWire onewire(temperaturesensor);
DallasTemperature sensor(&onewire);

float Ax, Ay, Az;

#define ANALOGTHRESHOLD 500
const int ShortPressTime = 400;
int lastState = LOW;
int currentState;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
float percent;

Adafruit_SSD1306 oled(WIDTH, HEIGHT, &Wire, OLED_RESET);

#define WIFI_TIMEOUT 20000
int status = WL_IDLE_STATUS;
WiFiServer server(80);
WiFiUDP udp;

// This is for the microphone
static const char channels = 1;
static const int frequency = 16000;
short sampleBuffer[512];
volatile int sampleRead;

bool powerOn = false; // status of the power button

#define EEPROM_SIZE 96
#define SSID_ADDR 0
#define PASS_ADDR 32

char ssid[32];
char password[32];

const int mqtt_port = 8883;
const char* mqtt_server = "mqtt.example.com";
const char* mqtt_username = "jacintabadu";
const char* mqtt_password = "JaCin@18";

WiFiClient espClient;
PubSubClient client(espClient);

const char* petcollar = "petcollar";
const char* temperaturetopic = "temperature";
const char* activitytopic = "activitylevel";
const char* heartratetopic = "heartrate";
const char* respiratorytopic = "respiratory";
const char* GPSTracker = "GPSTracker";


float batterylevel() {
  long total = 0;
  float voltage = 0.0;
  float output = 0.0;
  const float max_battery = 3.6;
  const float min_battery = 3.3;

  float R1 = 100000.0;
  float R2 = 10000.0;

  voltage = total / (float)500;
  voltage = voltage * (R1 + R2) / R2 * 3.3 / 4095;
  voltage = roundf(voltage * 100) / 100;
  Serial.print("Voltage: ");
  Serial.println(voltage, 2);
  output = ((voltage - min_battery) / (max_battery - min_battery)) * 100;
  return output < 100 ? output : 100.0f;
}

void drawBatteryIcon(int batteryLevel) {
  oled.drawRect(0, 20, 30, 15, WHITE);
  oled.fillRect(30, 23, 5, 10, WHITE);
  int fillwidth = map(batteryLevel, 0, 100, 0, 26);
  oled.fillRect(2, 22, fillwidth, 12, WHITE);
}

// void setRGBColor(int Red, int Green, int Blue) {
//   analogWrite(Pin_Red, Red);
//   analogWrite(Pin_Green, Green);
//   analogWrite(Pin_Blue, Blue);
// }

// For temperature reading
void checkTemperature() {
  sensor.requestTemperatures();
  float temperature = sensor.getTempCByIndex(0);

  if (temperature != DEVICE_DISCONNECTED_C) {
    //publishTemperatureData(temperature);

    if (temperature < 37.8) {
   //   setRGBColor(0, 0, 255); // Blue for too low
    } else if (temperature <= 39.2) {
   //   setRGBColor(0, 255, 0); // Green for normal
    } else {
    //  setRGBColor(255, 0, 0); // Red for too high
    }

    Serial.print("Published Temperature Data: ");
    Serial.println(temperature);
  } else {
    Serial.println("Error reading temperature");
  }
}

void ActivityLevel() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);
    float activitylevel = sqrt(Ax*Ax +Ay*Ay +Az*Az);

    if(activitylevel < 1.5){
      activitylevel=0 ;// low activity level
    }
    else if ( activitylevel <3.0){
      activitylevel =1; // moderate activity 
    }
    else{
      activitylevel =2 ; // high activity
    }
    //publishAccelerometerData(Ax,Ay,Az,activitylevel);
    Serial.print("Accelerometer data: ");
    Serial.print(Ax);
    Serial.print(", ");
    Serial.print(Ay);
    Serial.print(", ");
    Serial.println(Az);
    Serial.println();
    Serial.println(activitylevel);
  }
  delay(500);
}

// void GPSDisplay() {
//   Serial.print("Location: ");
//   if (gps.location.isValid()) {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(",");
//     Serial.print(gps.location.lng(), 6);
//     if (gps.altitude.isValid()) {
//       Serial.print(", Altitude: ");
//       Serial.println(gps.altitude.meters());
//     } else {
//       Serial.println(", Altitude: INVALID");
//     }
//   } else {
//     Serial.println("Location: INVALID");
//   }

//   Serial.print("Speed: ");
//   if (gps.speed.isValid()) {
//     Serial.print(gps.speed.kmph());
//     Serial.println(" km/h");
//   } else {
//     Serial.println("INVALID");
//   }

//   Serial.print("Date/Time: ");
//   if (gps.date.isValid()) {
//     Serial.print(gps.date.month());
//     Serial.print("/");
//     Serial.print(gps.date.day());
//     Serial.print("/");
//     Serial.print(gps.date.year());
//   } else {
//     Serial.print("Invalid Date");
//   }

//   Serial.print(" ");
//   if (gps.time.isValid()) {
//     if (gps.time.hour() < 10) Serial.print("0");
//     Serial.print(gps.time.hour());
//     Serial.print(":");
//     if (gps.time.minute() < 10) Serial.print("0");
//     Serial.print(gps.time.minute());
//     Serial.print(":");
//     if (gps.time.second() < 10) Serial.print("0");
//     Serial.print(gps.time.second());
//     Serial.print(".");
//     if (gps.time.centisecond() < 10) Serial.print("0");
//     Serial.print(gps.time.centisecond());
//   } else {
//     Serial.print("Invalid Time");
//   }
//   Serial.println();
// }

// Heart Rate
void checkHeartRate() {
  pox.update();
  float heartRate = pox.getHeartRate();

  if (heartRate != 0) {
    //publishHeartRateData(heartRate);

    if (heartRate < 140) {
     // setRGBColor(0, 0, 255); // Blue for too low
    } else if (heartRate <= 220) {
   //   setRGBColor(0, 255, 0); // Green for normal
    } else {
    //  setRGBColor(255, 0, 0); // Red for too high
    }

    Serial.print("Published Heart Rate Data: ");
    Serial.println(heartRate);
  }
}

void onBeatDetected(){
  Serial.println("Beat!");
}

// void connectToWiFi(const char* ssid, const char* password) {
//   WiFi.begin(ssid, password);
//   Serial.print("Connecting to Wi-Fi");
//   unsigned long startAttemptTime = millis();
//   while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
//     delay(1000);
//     Serial.print(".");
//   }
//   if (WiFi.status() == WL_CONNECTED) {
//     Serial.println();
//     Serial.print("Connected! IP address: ");
//     Serial.println(WiFi.localIP());
//   } else {
//     Serial.println();
//     Serial.println("Failed to connect to Wi-Fi");
//   }
// }

// void reconnect() {
//   while (!client.connected()) {
//     Serial.print("Attempting MQTT Connection...");
//     String clientId = "Esp32Client-";
//     clientId += String(random(0xffff), HEX);
//     if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
//       Serial.println("Connected");
//       client.subscribe(temperaturetopic);
//       client.subscribe(activitytopic);
//       client.subscribe(heartratetopic);
//       client.subscribe(GPSTracker);
//     } else {
//       Serial.print("Failed, rc= ");
//       Serial.print(client.state());
//       Serial.println(" Trying again in 10 seconds");
//       delay(10000);
//     }
//   }
// }

// void sendForm(WiFiClient client) {
//   client.println("HTTP/1.1 200 OK");
//   client.println("Content-type:text/html");
//   client.println();
//   client.println("<html><body>");
//   client.println("<form method='POST' action='/save'>");
//   client.println("SSID: <input type='text' name='ssid'><br>");
//   client.println("Password: <input type='text' name='pass'><br>");
//   client.println("<input type='submit' value='Save'>");
//   client.println("</form>");
//   client.println("</body></html>");
// }

// For the voice command
// Function to control power with voice command
// void controlPowerWithVoice(String command) {
//   if(sampleRead){
//     for (int i=0; i< sampleRead; i++){
//       if(channels ==2){
//         Serial.print("L: ");
//         Serial.print(sampleBuffer[i]);
//         Serial.print("R : ");
//       }
//       Serial.println(sampleBuffer[i]);

//       if(sampleBuffer[i] > 1000 || sampleBuffer[i] <= -1000){
//         powerOn = !powerOn;
//         if(powerOn){
//           Serial.println();
//           digitalWrite(pressbutton, HIGH);
//           Serial.println ("Power Is ON!");
//           Serial.println();
//           delay(1000);
//         }
//         else{
//           Serial.println();
//           digitalWrite(pressbutton, LOW);
//           Serial.println ("Power Is ON!");
//           Serial.println();
//           delay(1000);
//         }
//       }
//     }

//     // Clearing the read count
//     sampleRead =0;
//   }
  // if (command == "on") {
  //   powerOn = true;
  //   digitalWrite(pressbutton, HIGH); // Turn relay on
  //   Serial.println("Power turned on by voice command");
  // } else if (command == "off") {
  //   powerOn = false;
  //   digitalWrite(pressbutton, LOW); // Turn relay off
  //   Serial.println("Power turned off by voice command");
  // }
//}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  sampleRead = bytesAvailable / 2;
}

// Function to publish accelerometer data
void publishAccelerometerData(float x, float y, float z,float activitylevel) {
  char message[100];
  snprintf(message, sizeof(message), "Accelerometer: X:%.2f,Y:%.2f,Z:%.2f,activitylevel:%.2f", x, y, z,activitylevel);
  client.publish("ActivityLevel", message);
}

// Function to publish heart rate data
void publishHeartRateData(float heartRate) {
  char message[50];
  snprintf(message, sizeof(message), "Heart Rate: %.2f bpm", heartRate);
  client.publish("HeartRate", message);
}

// Function to publish temperature data
void publishTemperatureData(float temperature) {
  char message[50];
  snprintf(message, sizeof(message), "Temperature: %.2f bpm", temperature);
  client.publish("Temperature", message);
}

// For the GPS
void publishGPSData(float lng,float lat, float speed, float altitude, float time) {
  char message[50];
  snprintf(message, sizeof(message), "GPS: %.2f bpm", lng);
  client.publish("GPS", message);
}

void handlePowerButton() {
  currentState = digitalRead(pressbutton);
  if (currentState == HIGH && lastState == LOW) {
    pressedTime = millis();
  } else if (currentState == LOW && lastState == HIGH) {
    releasedTime = millis();
    long pressDuration = releasedTime - pressedTime;
    if (pressDuration < ShortPressTime) {
      // Handle short press
      digitalWrite(pressbutton, LOW);
    }
  }
  lastState = currentState;
}

void setup() {
  Serial.begin(115200);
 // connectToWiFi();
  // client.setServer(mqtt_server, mqtt_port);

 // GPS_SERIAL.begin(GPS_BAUDRATE);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Setting the states of the pins
  pinMode(LDR, INPUT);
  pinMode(pressbutton, INPUT);
  pinMode(Pin_Red, OUTPUT);
  pinMode(Pin_Green, OUTPUT);
  pinMode(Pin_Blue, OUTPUT);

  // Initializing the sensors
  sensor.begin(); // DS18B20

  // PDM.onReceive(onPDMdata);
  // if (!PDM.begin(channels, frequency)) {
  //   Serial.println("Failed to start PDM!");
  //   while (1);
  // }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  // Initialing the sensor for the HeartRate
  // if (!pox.begin()) {
  //   Serial.println("Failed to initialize pulse oximeter!");
  //   while (1);
  // }
  // else{
  //   Serial.println("Success");
  // }
  // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  // Registeromg a callback routine
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // For the OLED display
  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println(F("SSD1306 allocation failed"));
    while(true);
  }
  delay(2000); // waiting for 2 seconds before intializing

  // Printing the wifi status
 // PrintWifiStatus();
}

void loop() {
  // handlePowerButton();
  // if (!client.connected()) {
  //   reconnect();
  // }
  // client.loop();
  // // For the heart rate sensor
  // pox.update(); 

  // if(millis()- LastTimeHeartBeat > REPORTING_PERIOD){
  //   checkHeartRate();
  //   LastTimeHeartBeat = millis();
  // }

  // For the temperature sensor 
  checkTemperature();

  ActivityLevel();

  delay(5000); // Adjust the delay as needed

  float battery = batterylevel();
  if (battery < 10) {
    Serial.println("Battery is going low");
  //  setRGBColor(255, 0, 0);
    oled.clearDisplay();
    drawBatteryIcon(battery);
    oled.display();
  } else if (battery == 100) {
    Serial.println("The battery is fully charged");
  //  setRGBColor(0, 255, 0);
    oled.clearDisplay();
    drawBatteryIcon(battery);
    oled.display();
  } else {
  //  setRGBColor(0, 0, 0);
    oled.clearDisplay();
    drawBatteryIcon(battery);
    oled.display();
  }

  // while (GPS_SERIAL.available() > 0) {
  //   if (gps.encode(GPS_SERIAL.read())) {
  //     GPSDisplay();
  //   }
  // }
  // if (millis() > 5000 && gps.charsProcessed() < 10) {
  //   Serial.println("No GPS Detected: check your wiring");
  //   while (true);
  // }

}

// void PrintWifiStatus(){
//   // Printing out the SSID of the network that is being used
//   Serial.print("SSID");
//   Serial.println(WiFi.SSID());

//   // Printing the WiFi shield IP address
//   IPAddress ip = WiFi.localIP();
//   Serial.print("IP Address : ");
//   Serial.println(ip);

//   // Printing the output
//   Serial.println("Open the website to use the ip address");
//   Serial.println(ip);

// }