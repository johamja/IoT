#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "ClosedCube_HDC1080.h"

ClosedCube_HDC1080 hdc1080;
/*
Este código de ejemplo muestra el uso normal de un objeto TinyGPSPlus (TinyGPSPlus). 
Requiere el uso de SoftwareSerial y asume que tiene un dispositivo GPS serie de 4800 
baudios conectado a los pines 4 (rx) y 3 (tx).
*/
static const int RXPin = 2, TXPin = 0;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsInput(RXPin, TXPin);

void smartDelay(unsigned long ms);

void printSerialNumber();

void setup() {

  Serial.begin(115200);
  gpsInput.begin(GPSBaud);

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPSPlus features"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
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
    while (gpsInput.available())
      gps.encode(gpsInput.read());
  } while (millis() - start < ms);
}

void printSerialNumber() {
	Serial.print("Device Serial Number=");
	HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
	char format[12];
	sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
	Serial.println(format);
}