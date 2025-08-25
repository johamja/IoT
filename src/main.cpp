#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "ClosedCube_HDC1080.h"

ClosedCube_HDC1080 hdc1080;
/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void smartDelay(unsigned long ms);

void printSerialNumber();

void setup() {

  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPSPlus features"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));

	Serial.println("ClosedCube HDC1080 Arduino Test");

	// Default settings: 
	//  - Heater off
	//  - 14 bit Temperature and Humidity Measurement Resolutions
	hdc1080.begin(0x40);

	Serial.print("Manufacturer ID=0x");
	Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
	Serial.print("Device ID=0x");
	Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
	
	printSerialNumber();

}


void loop() {
  smartDelay(100);
  
  Serial.print(gps.location.lng());
  Serial.print(" ");
  Serial.print(gps.location.lat());
  Serial.print(" ");
  Serial.print(gps.altitude.meters());
    Serial.print(" ");
  
  char sz[32];
  sprintf(sz, "%02d/%02d/%02d ", gps.date.month(), gps.date.day(), gps.date.year());
  Serial.print(sz);
  Serial.print(" ");

  Serial.print("T=");
	Serial.print(hdc1080.readTemperature());
	Serial.print("C, RH=");
	Serial.print(hdc1080.readHumidity());
	Serial.println("%");

}

// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void printSerialNumber() {
	Serial.print("Device Serial Number=");
	HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
	char format[12];
	sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
	Serial.println(format);
}