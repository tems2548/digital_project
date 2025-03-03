#include <Arduino.h>
#include <SoftwareSerial.h>                



SoftwareSerial ParticleSerial(16, 17); // RX, TX


unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;


//diff AQI2.5
int X1 = -25;
int X2 = 24;
int X3 = 49;
int X4 = 99;
//diff MIN MAX pm 2.5
int I1 = -25;
int I2 = 11;
int I3 = 12;
int I4 = 39;
//diff MIN MAX pm 10
int Y1 = -50;
int Y2 = 29;
int Y3 = 39;
int Y4 = 59;




int AQI2_5(int pm2_5){
  int _AQI_2_5;
  if(pm2_5 > 0 && pm2_5 < 26){
    _AQI_2_5 = ((X1/I1)*(pm2_5-0))+0;
  }else if(pm2_5 >= 26 && pm2_5 < 38){
    _AQI_2_5 = ((X2/I2)*(pm2_5-26))+26;
  }else if(pm2_5 >= 38 && pm2_5 < 51){
    _AQI_2_5 = ((X3/I3)*(pm2_5-38))+51;
  }else if(pm2_5 >= 51 && pm2_5 < 91){
    _AQI_2_5 = ((X4/I4)*(pm2_5-51))+101;
  }else if(pm2_5 >= 92){
    Serial.print("danger AQI > 200");
  }
  return _AQI_2_5;
}
int AQI10_0(int pm10_0){
  int _AQI_10_0;
    if(pm10_0 > 0 && pm10_0 < 51){
    _AQI_10_0 = ((X1/Y1)*(pm10_0-0))+0;
  }else if(pm10_0 >= 51 && pm10_0 < 81){
    _AQI_10_0 = ((X2/Y2)*(pm10_0-26))+26;
  }else if(pm10_0 >= 81 && pm10_0 < 121){
    _AQI_10_0 = ((X3/Y3)*(pm10_0-38))+51;
  }else if(pm10_0 >= 121 && pm10_0 < 181){
    _AQI_10_0 = ((X4/Y4)*(pm10_0-51))+101;
  }else if(pm10_0 >= 181){
    Serial.print("danger AQI > 200");
  }
  return _AQI_10_0;
}
int Thai_AQI(int PM2_5,int PM10){
  int final_AQI;
  int AQI_10_val = AQI10_0(PM10);
  int AQI_2_5_val = AQI2_5(PM2_5);
  if(AQI_10_val >= AQI_2_5_val){
    final_AQI = AQI_10_val;
  }else{
    final_AQI = AQI_2_5_val;
  }
  return final_AQI;
}
void read_pms_data(){
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
  delay(1000);
}
void setup()
{
  Serial.begin(115200);                                // *Imporant, Pass your Stream reference
  ParticleSerial.begin(9600);
}

void loop()
{
  // read_pms_data();
  // Serial.println("aqi = "+String(Thai_AQI(pm2_5,pm10)));
}