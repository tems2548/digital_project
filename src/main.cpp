#include <Arduino.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <Wire.h>

#include <ShiftRegister74HC595.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //adr

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <WiFiMulti.h>

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <PubSubClient.h>


// MQTT broker details


const char* mqttServer = "192.168.1.38";
const int mqttPort = 1883;
const char* mqttUser = "mqtttest";
const char* mqttPassword = "Boombox0906";

const char* temperatureTopic = "esp32/sensor/temperature";

WiFiClient espClient;
PubSubClient CLIENT(espClient);

ShiftRegister74HC595<1> sr(41, 39, 40);

//!------------------------------------------------------------------------------------//

// WiFi AP SSID and password
#define WIFI_SSID "0527S24ultra"
#define WIFI_PASSWORD "3.1415926535"

#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "o7cDrNnFm6W8T0HfVeY-ENPH7k5V-DVSQ4w9uueHSn6z5cUI6nK4GCLfyn04ktdVSIVInyvDCmqJ7Mb0-DYzvg=="
#define INFLUXDB_ORG "561bba456237771c"
#define INFLUXDB_BUCKET "Home"

// Time zone info
#define TZ_INFO "UTC7"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensor("Sensor_Data");

#define DEVICE "ESP32_S3"
#define INFLUXDB_SEND_TIME (10000u)
uint32_t influxdb_timestamp = 5000;
//!------------------------------------------------------------------------------------//

#define PMS_Tx 16
#define PMS_Rx 17

const int BINARY_PIN[] = {39, 40, 41, 42}; // Define the pins
const int numLeds = sizeof(BINARY_PIN) / sizeof(BINARY_PIN[0]); // Calculate the number of LEDs

#define BAUDRATE 9600

SoftwareSerial ParticleSerial(PMS_Rx, PMS_Tx); // RX, TX
WiFiMulti wifiMulti;
SCD4x SCD;
Adafruit_BMP280 bmp;


unsigned int pm1_0 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;
int temperature;
int humidity;
int pressure;
int CO2;

unsigned long period = 2000; //wait time
unsigned long last_time = 0;

// diff AQI2.5
int X1 = -25;
int X2 = 24;
int X3 = 49;
int X4 = 99;
// diff MIN MAX pm 2.5
int I1 = -25;
int I2 = 11;
int I3 = 12;
int I4 = 39;
// diff MIN MAX pm 10
int Y1 = -50;
int Y2 = 29;
int Y3 = 39;
int Y4 = 59;

int AQI2_5(int pm2_5)
{
  int _AQI_2_5;
  if (pm2_5 > 0 && pm2_5 < 26)
  {
    _AQI_2_5 = ((X1 / I1) * (pm2_5 - 0)) + 0;
  }
  else if (pm2_5 >= 26 && pm2_5 < 38)
  {
    _AQI_2_5 = ((X2 / I2) * (pm2_5 - 26)) + 26;
  }
  else if (pm2_5 >= 38 && pm2_5 < 51)
  {
    _AQI_2_5 = ((X3 / I3) * (pm2_5 - 38)) + 51;
  }
  else if (pm2_5 >= 51 && pm2_5 < 91)
  {
    _AQI_2_5 = ((X4 / I4) * (pm2_5 - 51)) + 101;
  }
  else if (pm2_5 >= 92)
  {
    Serial.print("danger AQI > 200");
  }
  return _AQI_2_5;
}
int AQI10_0(int pm10_0)
{
  int _AQI_10_0;
  if (pm10_0 > 0 && pm10_0 < 51)
  {
    _AQI_10_0 = ((X1 / Y1) * (pm10_0 - 0)) + 0;
  }
  else if (pm10_0 >= 51 && pm10_0 < 81)
  {
    _AQI_10_0 = ((X2 / Y2) * (pm10_0 - 26)) + 26;
  }
  else if (pm10_0 >= 81 && pm10_0 < 121)
  {
    _AQI_10_0 = ((X3 / Y3) * (pm10_0 - 38)) + 51;
  }
  else if (pm10_0 >= 121 && pm10_0 < 181)
  {
    _AQI_10_0 = ((X4 / Y4) * (pm10_0 - 51)) + 101;
  }
  else if (pm10_0 >= 181)
  {
    Serial.print("danger AQI > 200");
  }
  return _AQI_10_0;
}
int Thai_AQI(int PM2_5, int PM10)
{
  int final_AQI;
  int AQI_10_val = AQI10_0(PM10);
  int AQI_2_5_val = AQI2_5(PM2_5);
  if (AQI_10_val >= AQI_2_5_val)
  {
    final_AQI = AQI_10_val;
  }
  else
  {
    final_AQI = AQI_2_5_val;
  }
  return final_AQI;
}

void read_pms_data()
{
  int index = 0;
  char value;
  char previousValue;

  while (ParticleSerial.available())
  {
    value = ParticleSerial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d))
    {
      Serial.println("Cannot find the data header.");
      break;
    }

    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14)
    {
      previousValue = value;
    }
    else if (index == 5)
    {
      pm1_0 = 256 * previousValue + value;
      // Serial.print("{ ");
      // Serial.print("\"pm1\": ");
      // Serial.print(pm1);
      // Serial.print(" ug/m3");
      // Serial.print(", ");
    }
    else if (index == 7)
    {
      pm2_5 = 256 * previousValue + value;
      // Serial.print("\"pm2_5\": ");
      // Serial.print(pm2_5);
      // Serial.print(" ug/m3");
      // Serial.print(", ");
    }
    else if (index == 9)
    {
      pm10 = 256 * previousValue + value;
      // Serial.print("\"pm10\": ");
      // Serial.print(pm10);
      // Serial.print(" ug/m3");
    }
    else if (index > 15)
    {
      break;
    }
    index++;
  }
  while (ParticleSerial.available())
    ParticleSerial.read();
}
void Read_CO2_DATA()
{
  if (SCD.readMeasurement()) // readMeasurement will return true when fresh data is available
  {
    Serial.println();

    Serial.print(F("CO2(ppm):"));
    Serial.print(SCD.getCO2());

    Serial.print(F("\tTemperature(C):"));
    Serial.print(SCD.getTemperature(), 1);

    Serial.print(F("\tHumidity(%RH):"));
    Serial.print(SCD.getHumidity(), 1);

    Serial.println();
  }

  temperature = SCD.getTemperature();
  humidity = SCD.getHumidity();
  CO2 = SCD.getCO2();

}
void Read_BMP_280()
{
  if (bmp.takeForcedMeasurement())
  {
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure() / 100);
    Serial.println(" hPa");
    
    pressure = bmp.readPressure() / 100;
    // Serial.print(F("Approx altitude = "));
    // Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    // Serial.println(" m");

    Serial.println();
  }
  else
  {
    Serial.println("Forced measurement failed!");
  }
}
void Wifi_Setup()
{
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
}
void INFLUXDB_TASK_INIT()
{
  sensor.addTag("device", DEVICE);
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov"); // sync time
  if (client.validateConnection())
  {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  }
  else
  {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}
void INFLUXDB_TASK_MNG()
{
  uint32_t now = millis();

  // if(now - influxdb_timestamp >= 0){
  influxdb_timestamp = now;
  sensor.clearFields();

  // REPORT RSSI
  sensor.addField("AQI", Thai_AQI(pm2_5,pm10));
  sensor.addField("HUMIDITY", humidity);
  sensor.addField("TEMPERATURE", temperature);
  sensor.addField("PRESSURE", pressure);
  sensor.addField("CARBONDIOXIDE", CO2);

  sensor.addField("PARTICLE[10]", pm10);
  sensor.addField("PARTICLE[2.5]", pm2_5);
  sensor.addField("PARTICLE[1.0]", pm1_0);

  
  if (!client.writePoint(sensor))
  {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  Serial.println(client.pointToLineProtocol(sensor));
  //}
}
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // if (String(topic) == "arduino/switch/control") {
  //   if (messageTemp == "ON") {
  //     digitalWrite(ledPin, HIGH);
  //     ledState = true;
  //   } else if (messageTemp == "OFF") {
  //     digitalWrite(ledPin, LOW);
  //     ledState = false;
  //   }
  //   CLIENT.publish("arduino/switch/status", ledState ? "ON" : "OFF");
  }

  void reconnect() {
    while (!CLIENT.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (CLIENT.connect("ESP32CLIENT", mqttUser, mqttPassword)) { // Include user/pass if needed
        Serial.println("connected");
      } else {
        Serial.print("failed, rc=");
        Serial.print(CLIENT.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }
  
void setup_MQTT(){
  // Connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  CLIENT.setServer(mqttServer, mqttPort);
  reconnect(); 
}
void setup()
{
  Serial.begin(115200); // *Imporant, Pass your Stream reference

  for (int i = 0; i < numLeds; i++) {
    pinMode(BINARY_PIN[i], OUTPUT);
  }
  
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // //!TEST
  // for (int i = 0; i < numLeds; i++) {
  //   digitalWrite(BINARY_PIN[i], HIGH);
  //   delay()
  // }
  // delay(2000); // Wait 1 second

  // // Turn all LEDs off
  // for (int i = 0; i < numLeds; i++) {
  //   digitalWrite(BINARY_PIN[i], LOW);
  //   delay(400);
  // }

  ParticleSerial.begin(BAUDRATE);

  Wire.begin();
  bmp.begin();
  SCD.begin();

  Wifi_Setup();
  INFLUXDB_TASK_INIT();
  //setup_MQTT();

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void displayONE(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  //temp
  display.setCursor(4,4);
  display.print("TEMP"); 

  display.setCursor(53,4);
  display.print(":"); 

  display.setCursor(77,4);
  display.print(temperature); 

  display.setCursor(103,0);
  display.print(char(248));
  display.setCursor(109,4);
  display.print("C");

  //humidity
  display.setCursor(4,16);
  display.print("HUMIDITY"); 

  display.setCursor(53,16);
  display.print(":"); 

  display.setCursor(77,16);
  display.print(humidity); 

  display.setCursor(109,16);
  display.print("%");

  //pressure
  display.setCursor(4,28);
  display.print("PRESSURE"); 

  display.setCursor(53,28);
  display.print(":"); 

  display.setCursor(77,28);
  display.print(pressure); 

  display.setCursor(109,28);
  display.print("hPa");

  // CO2
  display.setCursor(4,40);
  display.print("CO2");
  
  display.setCursor(53,40);
  display.print(":");

  display.setCursor(77,40);
  display.print(CO2); 

  display.setCursor(109,40);
  display.print("PPM");

  // display.setCursor(74,49);
  // display.print("|");

  //AQI
  display.setCursor(4,52);
  display.print("AQI");

  display.setCursor(53,52);
  display.print(":");

  display.setCursor(77,52);
  display.print(Thai_AQI(pm2_5,pm10));
}

void displayTWO(){
  int voltage = 220 ;
  int current = 99 ;
  int power = 9000;
  int frequency = 50;
  int energy = 1 ;

  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  //voltage
  display.setCursor(4,4);
  display.print("VOLTAGE"); 

  display.setCursor(58,4);
  display.print(":"); 

  display.setCursor(80,4);
  display.print(voltage); 

  display.setCursor(115,4);
  display.print("V");

  //current
  display.setCursor(4,16);
  display.print("CURRENT"); 

  display.setCursor(58,16);
  display.print(":"); 

  display.setCursor(80,16);
  display.print(current); 

  display.setCursor(115,16);
  display.print("A");

  //power
  display.setCursor(4,28);
  display.print("POWER"); 

  display.setCursor(58,28);
  display.print(":"); 

  display.setCursor(80,28);
  display.print(power); 

  display.setCursor(115,28);
  display.print("W");

  //frequency
  display.setCursor(4,40);
  display.print("FREQUENCY"); 

  display.setCursor(58,40);
  display.print(":"); 

  display.setCursor(80,40);
  display.print(frequency); 

  display.setCursor(115,40);
  display.print("Hz");

  //energy
  display.setCursor(4,52);
  display.print("ENERGY"); 

  display.setCursor(58,52);
  display.print(":"); 

  display.setCursor(80,52);
  display.print(energy); 

  display.setCursor(115,52);
  display.print("J");
}

void loop()
{
  displayONE();
  //displayTWO();
  
  if( millis() - last_time > period) {

    last_time = millis(); 

    Read_CO2_DATA();
    Read_BMP_280();
    read_pms_data();
    // for (int i = 0; i < 8; i++) {
    
    //   sr.set(i, HIGH); // set single pin HIGH
    //   delay(250); 
    // }
    INFLUXDB_TASK_MNG();
    // sr.setAllLow();
    // delay(250);
    // sr.updateRegisters();
   // CLIENT.publish(temperatureTopic, String(temperature).c_str());

  //  temperature = SCD.getTemperature();
  // humidity = SCD.getHumidity();
  // CO2 = SCD.getCO2();
}

//   if(!CLIENT.connected()) {
//   reconnect();
// }
//   CLIENT.loop();
display.display();

}