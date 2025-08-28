#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "ClosedCube_HDC1080.h"
#include <ESP8266WiFi.h>
#include <list>

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

// Configuracion al interner
// const char *ssid = "MyWiFi";
// const char *password = "12345678";
const char *ssid = "UPBWiFi";
const char *password = "";
// poner la direccion IP del servidor
const char *server = "192.168.0.101";
double latitud = 6.2455678;
double longitud = -75.4678888;
int id = 1001;

void smartDelay(unsigned long ms);

void printSerialNumber();

static void printStr(const char *str, int len);

static void printInt(unsigned long val, bool valid, int len);

// leer 3 segundos

// Crea una lista de enteros
std::list<double> list_temp;
std::list<double> list_h;

void setup() {
    // Dejar al inicio porque se necesita para los mensajes
    //  No mover
    Serial.begin(115200);

    // Conexion al wifi
    WiFiClient client;

    delay(10);
    pinMode(A0, INPUT);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    //--- ---- ---- --- --- -- --- -

    gpsInput.begin(GPSBaud);

    Serial.println(F("FullExample.ino"));
    Serial.println(F("An extensive example of many interesting TinyGPSPlus features"));
    Serial.print(F("Testing TinyGPSPlus library v. "));
    Serial.println(TinyGPSPlus::libraryVersion());
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
    // Para mandar a dormir usamos el dilay


    smartDelay(100);

    Serial.print(gps.location.lng());
    Serial.print(" ");
    Serial.print(gps.location.lat());
    Serial.print(" ");
    Serial.print(gps.altitude.meters());
    Serial.print(" ");

    printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
    printInt(gps.failedChecksum(), true, 9);

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

void printSerialNumber() {
    Serial.print("Device Serial Number=");
    HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
    char format[12];
    sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
    Serial.println(format);
}

// Chirp
// Recollecion de los datos de los dos sensores
// GPS y Sensor de temperatura
/**
 * proceso de tomar el chirp de muestras de un sensor,
 * y concentrarlo en un procesador de un end device
 */

void chrip_temp_and_h(int ms, std::list<double> *list, ClosedCube_HDC1080 hdc1080) {
    int contador = millis();
    while (contador < ms) {
        list_temp.push_back(hdc1080.readTemperature());
        list_h.push_back(hdc1080.readHumidity());
    }
}


// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (gpsInput.available())
            gps.encode(gpsInput.read());
    } while (millis() - start < ms);
}

// Pruning
/**
 * proceso de reducir el número de muestras del chirp de medidas,
 * aplicando técnicas de estimación, corrección/calibración,
 * teorema central del límite, para determinar el valor medido en el proceso de sensado
 */

// Bundling
/**
 * proceso de construir la información para enviar por IPv6 o IPv4 hacia o desde el Gateway,
 * esta función la puede o no hacer el End Device, también se puede hacer en el collector
 * o en el mismo gateaway, todo depende de la arquitectura de hardware escogida
 * (puede ser un único dispositivo el que hace todo, o puede estar distribuidas las funciones en la arquitectura)
 */


// Collection
// Enviar el packete

static void printStr(const char *str, int len) {
    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        Serial.print(i < slen ? str[i] : ' ');
    smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    Serial.print(sz);
    smartDelay(0);
}
