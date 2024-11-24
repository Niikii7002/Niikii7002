#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// Instancias de sensores y Wi-Fi
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;
WiFiClient client;

// Configuración de Wi-Fi
const char* ssid = "Redmi 9";
const char* password = "17846547";

// Configuración de ThingSpeak
const char* apiKey = "G41Z37S3VO0F3M9E"; // Tu API Key
unsigned long myChannelNumber = 2722684; // Reemplaza con tu número de canal

// Almacenamiento para frecuencia cardíaca
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicialización del sensor de frecuencia cardíaca MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 no encontrado. Verifique las conexiones.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0F);
  particleSensor.setPulseAmplitudeGreen(0);

  // Inicialización del sensor MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 no encontrado. Verifique las conexiones.");
    while (1);
  }

  // Conectar a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a la red Wi-Fi.");

  // Inicializar ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
  // Leer datos del sensor de frecuencia cardíaca
  long irValue = particleSensor.getIR();
  
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }

  // Mostrar datos de frecuencia cardíaca
  if (irValue < 50000) {
    Serial.println("Por favor, coloque su dedo en el sensor");
  } else {
    Serial.print("Latidos por minuto: ");
    Serial.println(beatsporminuto);
    Serial.print("Promedio BPM: ");
    Serial.println(beats);
  }

  // Leer datos del sensor MPU6050
  sensors_event_t accel, gyro;
  mpu.getAccelerometerSensor()->getEvent(&accel);
  mpu.getGyroSensor()->getEvent(&gyro);

  // Imprimir los valores de acelerómetro y giroscopio
  float accX = accel.acceleration.x;
  float accY = accel.acceleration.y;
  float accZ = accel.acceleration.z;
  float gyroX = gyro.gyro.x;
  float gyroY = gyro.gyro.y;
  float gyroZ = gyro.gyro.z;

  Serial.print("Acelerómetro [X]: "); Serial.print(accX);
  Serial.print(" [Y]: "); Serial.print(accY);
  Serial.print(" [Z]: "); Serial.println(accZ);

  Serial.print("Giroscopio [X]: "); Serial.print(gyroX);
  Serial.print(" [Y]: "); Serial.print(gyroY);
  Serial.print(" [Z]: "); Serial.println(gyroZ);

  // Enviar datos a ThingSpeak
  ThingSpeak.setField(1, beatsPerMinute); // BPM
  ThingSpeak.setField(2, beatAvg); // Promedio de BPM
  ThingSpeak.setField(3, accX); // Acelerómetro X
  ThingSpeak.setField(4, accY); // Acelerómetro Y
  ThingSpeak.setField(5, accZ); // Acelerómetro Z
  ThingSpeak.setField(6, gyroX); // Giroscopio X
  ThingSpeak.setField(7, gyroY); // Giroscopio Y
  ThingSpeak.setField(8, gyroZ); // Giroscopio Z

  // Publicar en ThingSpeak
  int responseCode = ThingSpeak.writeFields(myChannelNumber, apiKey);
  
  if (responseCode == 200) {
    Serial.println("Datos enviados a ThingSpeak exitosamente.");
  } else {
    Serial.print("Error al enviar datos a ThingSpeak. Código de respuesta: ");
    Serial.println(responseCode);
  }

  delay(5000); // Enviar datos cada 5 segundos
}
