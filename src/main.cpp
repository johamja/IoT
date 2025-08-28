#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "ClosedCube_HDC1080.h"
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <map>
#include <mbedtls/sha256.h> // Para el hash SHA-256
#include <base64.h>			// Para codificar el cifrado (simulación)
#include <time.h>			// Para timestamp
#include <ArduinoJson.h>
#include "mbedtls/aes.h"
#include <Hash.h>

// Constantes para usar WiFi
const char *ssid = "UPBWiFi";
const char *password = "";
const char *route_server = "localhost:8080/iot";
WiFiClient client;

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

// Constantes para el Chirp y el Pruning
// const char *aesKey = "MiClaveAES128Key"; // Simulación de clave AES (en la práctica usa librerías AES reales)
// Vector de inicialización (16 bytes, puede generarse aleatoriamente)
// unsigned char iv[16] = {
//	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
//	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
String output;
int sample = 10;				 // Número de muestras a tomar
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
{ return a < b; };									// Comparador para ordenar de menor a mayor
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

	// 3
	gpsInput.begin(GPSBaud);

	// 4
	hdc1080.begin(0x40);
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

		// Crear payload crudo
		String rawPayload = "{";
		rawPayload += "\"temperature\":" + String(temperature_avg) + ",";
		rawPayload += "\"humidity\":" + String(humidity_avg) + ",";
		rawPayload += "\"latitude\":" + String(latitud) + ",";
		rawPayload += "\"longitude\":" + String(longitud);
		rawPayload += "}";

		// Cifrar payload real
		// String encryptedPayload = encryptAES128CBC(rawPayload);

		// Generar hash SHA-256 sobre el payload crudo
		// String hash = generateSHA256(rawPayload);
		
		// Obtener timestamp actual
		time_t now = time(nullptr);
		struct tm *timeinfo = gmtime(&now);
		char timestamp[30];
		strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", timeinfo);

		// Crear JSON final
		DynamicJsonDocument doc(512);
		JsonObject preamble = doc.createNestedObject("preamble");
		preamble["device_id"] = id;
		preamble["type"] = type;
		preamble["version"] = "1.0";
		preamble["timestamp"] = timestamp;

		doc["payload"] = rawPayload;

		JsonObject deambulo = doc.createNestedObject("deambulo");
		deambulo["token"] = 123134311;

		serializeJson(doc, output);

		Serial.println("Paquete final:");
		Serial.println(output);
		state = 6;
	}
	break;

	case 6:
	{
		Serial.println("Enviando la trama");
		HTTPClient http;

		http.begin(client, route_server);
		Serial.printf("[HTTP] begin... %s \n",route_server);

		http.POST(output);
		http.end();
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

	// SLEEP -> Esperar 10 segundos para la siguiente lectura
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

// Función para rellenar a múltiplos de 16 bytes (PKCS7 Padding)
/** String padData(const String &data)
{
	size_t padLen = 16 - (data.length() % 16);
	String padded = data;
	for (size_t i = 0; i < padLen; i++)
	{
		padded += (char)padLen;
	}
	return padded;
}
*/

/**
 * @brief Cifra una cadena de texto utilizando el algoritmo AES-128 en modo CBC.
 *
 * Este método aplica un cifrado simétrico para proteger la confidencialidad de los datos:
 * - Se utiliza una clave de 128 bits (16 bytes) definida previamente.
 * - Se aplica relleno PKCS7 para asegurar que la longitud sea múltiplo de 16 bytes.
 * - El cifrado se realiza bloque a bloque en modo CBC (Cipher Block Chaining),
 *   donde cada bloque se mezcla con el bloque cifrado anterior usando XOR.
 * - El resultado final se codifica en Base64 para poder ser incluido en un JSON.
 *
 * @param plainText Cadena de texto original (payload sin cifrar).
 * @return Cadena cifrada y codificada en Base64.
 */
/**
 String encryptAES128CBC(const String &plainText)
{
	String padded = padData(plainText);
	size_t len = padded.length();
	unsigned char output[len];

	mbedtls_aes_context aes;
	mbedtls_aes_init(&aes);
	mbedtls_aes_setkey_enc(&aes, (const unsigned char *)aesKey, 128);
	mbedtls_aes_free(&aes);

	// Codificamos en Base64 para enviar como texto
	return base64::encode(output, len);
}
*/

/**
 * @brief Genera un hash SHA-256 a partir de una cadena de texto.
 *
 * Esta función calcula la huella digital criptográfica de los datos originales:
 * - Utiliza el algoritmo SHA-256 (256 bits) para producir un hash de 64 caracteres hexadecimales.
 * - Es un proceso unidireccional: no se puede obtener el mensaje original a partir del hash.
 * - Se usa para verificar la integridad de los datos transmitidos.
 *
 * @param data Cadena de texto original sobre la que se calculará el hash.
 * @return Cadena hexadecimal de 64 caracteres representando el hash.
 */
/**
String generateSHA256(const String &data)
{
	br_hash_class_ hashHashSha256 sha256;
	sha256.begin();
	sha256.add(input.c_str(), input.length());
	uint8_t* hash = sha256.result();
	String hashString;
	for (int i = 0; i < 32; i++) {
		hashString += String(hash[i], HEX);
	}
	return hashString;
}
*/