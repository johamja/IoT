#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "ClosedCube_HDC1080.h"
#include <ESP8266WiFi.h>
#include <WiFiClientSecureBearSSL.h>
#include <map>
#include <mbedtls/sha256.h> // Para el hash SHA-256
#include <base64.h>         // Para codificar el cifrado (simulación)
#include <time.h>           // Para timestamp
#include <ArduinoJson.h>
#include "mbedtls/aes.h"
#include <Hash.h>

// Constantes para usar WiFi
const char *ssid = "joham";
const char *password = "joham1015";
const char *url = "10.199.26.8";
const int httpsPort = 443;
// SHA1 Fingerprint del certificado
const char fingerprint[] PROGMEM = "F4 FD EB 03 71 10 48 F0 D7 CA A0 1A FD 14 8B 65 D8 91 3D FC";
// Pegar aquí tu certificado PEM
static const char digicert_root_ca[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIID4zCCAsugAwIBAgIUBU7/ESsV9/fNwuEJbS+xBJvmokYwDQYJKoZIhvcNAQEL
BQAwgYAxCzAJBgNVBAYTAkNPMRMwEQYDVQQIDApTb21lLVN0YXRlMREwDwYDVQQH
DAhtZWRlbGxpbjEMMAoGA1UECgwDdXBiMQwwCgYDVQQLDAN1cGIxDDAKBgNVBAMM
A2lvdDEfMB0GCSqGSIb3DQEJARYQbGFiaW90QGdtYWlsLmNvbTAeFw0yNTA4Mjkx
ODA2MThaFw0yNjA4MjkxODA2MThaMIGAMQswCQYDVQQGEwJDTzETMBEGA1UECAwK
U29tZS1TdGF0ZTERMA8GA1UEBwwIbWVkZWxsaW4xDDAKBgNVBAoMA3VwYjEMMAoG
A1UECwwDdXBiMQwwCgYDVQQDDANpb3QxHzAdBgkqhkiG9w0BCQEWEGxhYmlvdEBn
bWFpbC5jb20wggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQC0ko+SqSCd
Bgs9t3ID0GuNpbuehIz+sFqMK5q5PoZQpNylYRLSXiJEMNTYbHc0SR/MipLrXPZU
CjuQ2IuyzyA+rO1TakDy3UYhq8sAkTE3Ey7Z9m0N235uxeShoPQBc6yeeJ/S/P1n
LqPeSFVqkOvy3GyyOidxRkAcr+lPtcq4fM2FrW5/GibVzrPq+viXv8AyGi7QNUo3
jM7/mSK9pctOS67gZUzkNiY3KGvPEopgp9TOVjNb/Yv56wY0hFizzKIxbDoztW7d
TdxCIQbbCtgaZSHL2pzV4Cuau1mwys0BPMtA5Ve3u3GbYA1vZIRgmjLQPtWfnE96
t/pTGn1CBu3LAgMBAAGjUzBRMB0GA1UdDgQWBBRvW+eQJtT7T31WLqBUvnF1NqR8
SzAfBgNVHSMEGDAWgBRvW+eQJtT7T31WLqBUvnF1NqR8SzAPBgNVHRMBAf8EBTAD
AQH/MA0GCSqGSIb3DQEBCwUAA4IBAQCWhbquKbUkV+rCwsv3jTid7VxJqHhVSYTg
Z/YdwUhih+j74eIsbVmZFcTkzVr2SHUcQ+4BvVL5yFojXu4CYFA65EMpRbVOB1+x
dXsEmF1REAB4lPjqsblWuim93Pf/w/JBQrDe1GuKBWowRIZKUPUVkLdKDOlcBHcA
/uvvh8y6MWhRw1g2t2+QgKTwNLC/PkWDmoWzI8gYBt5c8ve0euVd3EsgNxToXYCV
OINRhDPVobBjzOTc5lOVyA1Mefcafk7Qzrgmjlmnj2NlNR2eOitkdc4z6L2LBNGj
TNQUDOYrVJfIR1/oUU90W2RdxeRuAEgdjy8P6iTJO4bdEXuW8EBu
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure client;

// Constates para usar el GPS
static constexpr int RXPin = 2, TXPin = 0;
static constexpr uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsInput(RXPin, TXPin);

// Constantes para usar el sensor de temperatura y humedad
ClosedCube_HDC1080 hdc1080;

// Constates del dispositivo
const char *id = "device-01";
const char *type = "sensor-humedad-temperatura";

String output;
int sample = 10;                 // Número de muestras a tomar
int time_between_samples = 1000; // Tiempo entre muestras en milisegundos
double temperature_list[10];
double humidity_list[10];
double temperature_avg = 0;
double humidity_avg = 0;
double latitud = 0;
double longitud = 0;

struct Datos
{
    double longitud;
    double latitud;
};

auto cmp = [](const double a, const double b)
{ return a < b; };                                  // Comparador para ordenar de menor a mayor
std::map<double, Datos, decltype(cmp)> miMapa(cmp); // Mapa con llave int y valor Datos, ordenado de mayor a menor

// Declaración de metodos
void smartDelay(const unsigned long ms);

String encryptAES128CBC(const String &plainText);

String generateSHA256(const String &data);

/**
 * Configuración inicial para el servidor
 * 1. Configuración del serial begin en 115200
 * 2. Establecer una conexión WiFi
 * 3. Configurar el serial begin del GPS
 * 4. Configurar el serial begin del Sensor de Temperatura y Humedad
 */
void setup()
{

    // 4
    hdc1080.begin(0x40);

    // 1
    Serial.begin(115200);

    // 2
    Serial.println("Connecting to " + *ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    BearSSL::X509List cert(digicert_root_ca);
    client.setTrustAnchors(&cert);
    client.setFingerprint(fingerprint);

    // 3
    gpsInput.begin(GPSBaud);
}

int state = 1;

/**
 * Maquína de estados
 * 1. CHIRP -> Lectura de la temperatura y humedad, 10 muestras cada 10 segundos
 * 2. PRUNING -> Calcular la media de las muestras de temperatura y humedad
 * 3. CHIRPS -> Obtener la latitud y longitud del GPS, 10 muestras cada 10 segundos
 * 4. PRUNING -> Validar que el error del GPS sea menor a 3 metros
 * 5. BUNDLING -> Crear el JSON con los datos obtenidos y enviarlos al servidor
 *                (Opcional) Cifrar el JSON con una clave simétrica
 *                (Opcional) Realizar un hash del JSON para verificar la integridad de los datos
 *       1. Genera el payload con los datos de sensores (temperatura, humedad, GPS).
 *       2. Cifra la carga útil usando AES-128 en modo CBC (Cipher Block Chaining),
 *          asegurando la confidencialidad de los datos transmitidos.
 *        - La clave utilizada es de 128 bits (16 bytes).
 *        - Se aplica relleno PKCS7 para alinear los datos a bloques de 16 bytes.
 *        - Los datos cifrados se codifican en Base64 para transporte en JSON.
 *       3. Calcula un hash SHA-256 sobre la carga útil original (antes del cifrado)
 *          para garantizar la integridad del mensaje.
 *       4. Construye un paquete JSON con la siguiente estructura:
 *        - "preamble": contiene metadatos (ID de dispositivo, tipo, versión, timestamp).
 *        - "payload": datos cifrados en Base64.
 *        - "deambulo": incluye el hash SHA-256.
 * 6. BUNDLING and COLLECTION -> Enviar el JSON al servidor
 * 7. SLEEP -> Esperar 10 segundos para la siguiente lectura
 */
void loop()
{
    switch (state)
    {
    case 1:
    {
        Serial.println("Iniciando CHIRP de temperatura y humedad");
        for (int i = 0; i < sample; i++)
        {
            temperature_list[i] = hdc1080.readTemperature();
            humidity_list[i] = hdc1080.readHumidity();
            Serial.printf("\tMuestra %d Temperatura %lf Humedad %lf \n", i, temperature_list[i], humidity_list[i]);
            delay(time_between_samples);
        }
        state = 2;
    }

    break;

    case 2:
    {
        Serial.println("Iniciando PRUNING de temperatura y humedad");
        double temperature_sum = 0;
        double humidity_sum = 0;
        for (int i = 0; i < sample; i++)
        {
            temperature_sum += temperature_list[i];
            humidity_sum += humidity_list[i];
        }
        temperature_avg = temperature_sum / sample;
        humidity_avg = humidity_sum / sample;
        Serial.println("Temperatura promedio: " + String(temperature_avg));
        Serial.println("Humedad promedio: " + String(humidity_avg));
        state = 3;
    }
    break;

    case 3:
    {
        Serial.println("Iniciando CHIRP del GPS");
        for (int i = 0; i < sample; i++)
        {
            smartDelay(time_between_samples);

            latitud = gps.location.lat();
            longitud = gps.location.lng();
            const double error = gps.hdop.hdop();
            const double estimatedError = error * 5.0; // 5 meters is a common base error

            Serial.printf("\t Muestra %d Lat %lf Lon %lf Error %lf\n", i, latitud, longitud, estimatedError);

            miMapa[estimatedError] = {longitud, latitud};
        }
        state = 4;
    }

    break;

    case 4:
    {
        Serial.println("Iniciando PRUNING del GPS");
        const double error = miMapa.begin()->first;
        latitud = miMapa.begin()->second.latitud;
        longitud = miMapa.begin()->second.longitud;

        Serial.println("Latitud: " + String(latitud));
        Serial.println("Longitud: " + String(longitud));
        Serial.println("Error estimado: " + String(error) + " metros");
        state = 5;
    }

    break;

    case 5:
    {

        Serial.println("Iniciando BUNDLING");

        JsonDocument data;
        data["id"] = "point14";
        data["lat"] = latitud;
        data["lon"] = longitud;
        data["temperatura"] = temperature_avg;
        data["humedad"] = humidity_avg;
        serializeJson(data, output);

        Serial.println("Paquete final:");
        Serial.println(output);
        state = 6;
    }
    break;

    case 6:
    {
        Serial.println("Enviando la trama");

        if (!client.connect(url, httpsPort))
        {
            Serial.println("Conexión fallida a servidor HTTPS");
            return;
        }

        Serial.println("conectado");
        client.println("POST /update_data HTTP/1.1");
        client.println("Host: 10.15.69.51");
        client.println("User-Agent: Arduino/1.0");
        client.println("Connection: close");
        client.println("Content-Type: application/json");
        client.print("Content-Length: ");
        client.println(output.length());
        client.println();       // Línea vacía entre encabezados y cuerpo
        client.println(output); // Envía el cuerpo

        // Leer respuesta del servidor
        while (client.connected() || client.available())
        {
            String line = client.readStringUntil('\n');
            Serial.println(line);
            if (line == "{\"success\": true}") break;
        }

        state = 7;
    }
    break;

    case 7:
    {
        Serial.println("Durmiendo 10 segundos");
        delay(10000);
        state = 1;
    }
    break;

    default:
        Serial.println("Opcion por defecto");
    }
}

// Metodos
void smartDelay(const unsigned long ms)
{
    const unsigned long start = millis();
    do
    {
        while (gpsInput.available())
            gps.encode(gpsInput.read());
    } while (millis() - start < ms);
}