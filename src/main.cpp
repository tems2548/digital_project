#include <Arduino.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "SparkFun_SCD4x_Arduino_Library.h"

#include <WiFiMulti.h>

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

//!------------------------------------------------------------------------------------//

// WiFi AP SSID and password
#define WIFI_SSID "pu_kgs"
#define WIFI_PASSWORD "0960698678"

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

#define BAUDRATE 9600

SoftwareSerial ParticleSerial(PMS_Rx, PMS_Tx); // RX, TX
WiFiMulti wifiMulti;
SCD4x SCD;
Adafruit_BMP280 bmp;


unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;
int temperature;
int humidity;
int CO2;

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
      pm1 = 256 * previousValue + value;
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
void INFLUXDB_TASK_MNG(int temp, int humid)
{
  uint32_t now = millis();

  // if(now - influxdb_timestamp >= 0){
  influxdb_timestamp = now;
  sensor.clearFields();

  // REPORT RSSI
  sensor.addField("RSSI", WiFi.RSSI());

  sensor.addField("HUMIDITY", humid);
  sensor.addField("TEMPERATURE", temp);

  if (!client.writePoint(sensor))
  {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  Serial.println(client.pointToLineProtocol(sensor));
  //}
}
void setup()
{
  Serial.begin(115200); // *Imporant, Pass your Stream reference
  ParticleSerial.begin(BAUDRATE);

  Wire.begin();
  bmp.begin();
  SCD.begin();

  Wifi_Setup();
  INFLUXDB_TASK_INIT();

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop()
{
  INFLUXDB_TASK_MNG(temperature,humidity);
  Read_CO2_DATA();


  // Read_BMP_280();
  // read_pms_data();
  //  Serial.println("aqi = "+String(Thai_AQI(pm2_5,pm10)));
  //  Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");
  //  Serial.print("Pressure: ");Serial.print(humidity.relative_humidity);Serial.println(" RH %");
  // delay(200);
  
}