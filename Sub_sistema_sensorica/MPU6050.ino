#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Espera a que el puerto serial esté listo

  Serial.println("Iniciando MPU6050 prueba de conexión...");

  // Intenta inicializar el MPU6050
  if (!mpu.begin()) {
    Serial.println("No se encontró un sensor MPU6050, ¡revisa tus conexiones!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 encontrado!");

  // Establecer rango de acelerómetro
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Rango de acelerometro establecido a: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("±2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("±4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("±8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("±16G");
    break;
  }
  Serial.println("");
  delay(1000);
}

void loop() {
  /* Lea los sensores */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Imprime los valores del acelerómetro con unidades */
  Serial.print("Acelerometro - X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" m/s², Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" m/s², Z: ");
  Serial.println(a.acceleration.z);
  Serial.println(" m/s²");

  /* Imprime los valores del giroscopio con unidades */
  Serial.print("Giroscopio - X: ");
  Serial.print(g.gyro.x);
  Serial.print(" °/s, Y: ");
  Serial.print(g.gyro.y);
  Serial.print(" °/s, Z: ");
  Serial.println(g.gyro.z);
  Serial.println(" °/s");

  delay(500); // Retardo de 100ms entre lecturas
}
