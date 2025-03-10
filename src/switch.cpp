// #include <Arduino.h>
// #include <WiFi.h>
// #include <WiFiManager.h>
// #include <NTPClient.h>

// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP,"pool.ntp.org",25200, 60000);

// int hour ;
// int hourset ;
// int minute ;
// int minuteset ;

// int pinset = 14;
// int pin[] = {0,1,2,3} ; // pinbinary b3 b2 b1 b0

// void setup() {
//     Serial.begin(115200);

//     WiFiManager wm;

//     bool res;
    
//     res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

//     if(!res) {
//         Serial.println("Failed to connect :(");
//     } 
//     else {
//         //if you get here you have connected to the WiFi    
//         Serial.println("connected :)");
//     }

//     for(int i = 0 ; i <= 3 ; i++ ){
//         pinMode(pin[i],OUTPUT);
//     }

//     setTime(18,0);
// }

// bool b0(int i){
//     if(i%2){
//         return 1 ;
//     }
//     else{
//         return 0 ;
//     }
// }

// bool b1(int i){
//     i = i/2;
//     if(i%2){
//         return 1 ;
//     }
//     else{
//         return 0 ;
//     }
// }

// bool b2(int i){
//     i = i/4 ;
//     if(i%2){
//         return 1 ;
//     }
//     else{
//         return 0 ;
//     }
// }

// bool b3(int i){
//     i = i/8 ;
//     if(i%2){
//         return 1 ;
//     }
//     else{
//         return 0 ;
//     }
// }

// void binaryOut(int i){
//     digitalWrite(pin[0],b0(i));
//     digitalWrite(pin[1],b1(i));
//     digitalWrite(pin[2],b2(i));
//     digitalWrite(pin[3],b3(i));
// }

// void setTime(int h , int m){
//     hourset = h ;
//     minuteset = m ;
    
// }

// bool sw(){
//     if(hour == hourset & minute == minuteset){
//         return true;
//     }
//     else {
//         return false;
//     }
// }

// void light(){
//     if(sw()){
//         printf("Light ON");
//     }
//     else{
//         printf("Light OFF");
//     }
// }

// void loop() {
//     timeClient.update();
//     hour = timeClient.getHours();
//     minute = timeClient.getMinutes();

//     light();
//     binaryOut(pinset);
// }