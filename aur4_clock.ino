/*

  AUR4 Clock
  RTC clock using NTP server to sync time

  Created using the Arduino Uno R4 Wifi example code - RTC_NTPSync, initially created by Sebastian Romero @sebromero  

 * Instructions:
 * 1. Change the WiFi credentials in the arduino_secrets.h file to match your WiFi network.
 * 2. Set the orientation using the #define ORIENTATION 0 or 1
*/

#include "led-matrix.h"
#include "Arduino_LED_Matrix.h"
#include "RTC.h"
#include <WiFiS3.h>
#include "arduino_secrets.h"

#define TIMEZONE_OFFSET_HOURS 2
#define ORIENTATION 1 // 0 (up is where the ESP32 is), 1 (up is where the Qwiic is)

unsigned long currentMillis;
unsigned long previousMillis = 0;

byte currentFrame[NO_OF_ROWS][NO_OF_COLS];
byte rotatedFrame[NO_OF_ROWS][NO_OF_COLS];

position first = {5, 0}; // position of first digit
position second = {0, 0}; // etc.
position third = {5, 7};
position fourth = {0, 7};

// please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)

constexpr unsigned int LOCAL_PORT = 2390;  // local port to listen for UDP packets
constexpr int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

int wifiStatus = WL_IDLE_STATUS;
IPAddress timeServer(162, 159, 200, 123); // pool.ntp.org NTP server
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP

ArduinoLEDMatrix matrix;

void setDigit(position digitPosition, const byte digit[][5]){
  for(byte r = 0; r < 3; r++){
    for(byte c = 0; c < 5; c++){
      currentFrame[r+digitPosition.row][c+digitPosition.col] = digit[r][c];
    }
  }
}

void rotateFrame(){
  for(byte r = 0; r < NO_OF_ROWS; r++){
    for(byte c = 0; c < NO_OF_COLS; c++){
      rotatedFrame[r][c] = currentFrame[NO_OF_ROWS-1-r][NO_OF_COLS-1-c];
    }
  }
  memcpy(currentFrame, rotatedFrame, sizeof rotatedFrame);
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectToWiFi(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifiStatus = WiFi.begin(ssid, pass);
  }
  delay(5000);
  Serial.println("Connected to WiFi");
  printWifiStatus();
}

/**
 * Calculates the current unix time, that is the time in seconds since Jan 1 1970.
 * It will try to get the time from the NTP server up to `maxTries` times,
 * then convert it to Unix time and return it.
 * You can optionally specify a time zone offset in hours that can be positive or negative.
*/
unsigned long getUnixTime(int8_t timeZoneOffsetHours = 0, uint8_t maxTries = 5){
  // Try up to `maxTries` times to get a timestamp from the NTP server, then give up.
  for (size_t i = 0; i < maxTries; i++){
    sendNTPpacket(timeServer); // send an NTP packet to a time server
    // wait to see if a reply is available
    delay(1000);

    if (Udp.parsePacket()) {
      Serial.println("Packet received.");
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      //or two words, long. First, extract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      
      // Combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;

      // Now convert NTP time into everyday time:
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      unsigned long secondsSince1970 = secsSince1900 - seventyYears + (timeZoneOffsetHours * 3600);
      return secondsSince1970;
    } else {
      Serial.println("Packet not received. Trying again.");
    }
  }
  return 0;
}

void updateTime(){
  yield();  
  Serial.println("\nStarting connection to NTP server...");
  auto unixTime = getUnixTime(TIMEZONE_OFFSET_HOURS, 25);
  Serial.print("Unix time = ");
  Serial.println(unixTime);
  if(unixTime == 0){
    unixTime = getUnixTime(2,25);
  }
  RTCTime timeToSet = RTCTime(unixTime);
  RTC.setTime(timeToSet);
  Serial.println("Time updated.");
}

void setup() {
  Serial.begin(9600);
  connectToWiFi();
  Udp.begin(LOCAL_PORT);

  RTC.begin();
  updateTime();
  matrix.begin();  
}

void loop() {  
  currentMillis = millis();
  if(currentMillis - previousMillis > 43200000){ // 1000 * 60 * 60 * 12
    updateTime();
    previousMillis = currentMillis;
  }

  RTCTime currentTime;
  RTC.getTime(currentTime);
  
  String hour = (String) currentTime.getHour();
  if (hour.length() == 1){
    hour = "0" + hour;
  }

  String minutes = (String) currentTime.getMinutes();
  if (minutes.length() == 1){
    minutes = "0" + minutes;
  }

  setDigit(first, digits[hour.substring(0,1).toInt()]);
  setDigit(second, digits[hour.substring(1).toInt()]);
  setDigit(third, digits[minutes.substring(0,1).toInt()]);
  setDigit(fourth, digits[minutes.substring(1).toInt()]);
  if (ORIENTATION == 1){
    rotateFrame();
  } 
  matrix.renderBitmap(currentFrame, NO_OF_ROWS, NO_OF_COLS);   

  delay(1000);   
}
