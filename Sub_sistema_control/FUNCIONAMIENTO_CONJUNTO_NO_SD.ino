#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp; // I2C
QMC5883LCompass compass;

int calibrationData[3][2] = {{INT_MAX, INT_MIN}, {INT_MAX, INT_MIN}, {INT_MAX, INT_MIN}};
bool calibrated = false;

SoftwareSerial GPS_Serial(4, 5); // RX, TX
TinyGPSPlus gps;

volatile float minutos, segundos;
volatile int grados, secs, mins;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    delay(10); // Esperar conexión serial

  if (!mpu.begin())
  {
    Serial.println("MPU6050 no encontrado!");
    while (1)
      delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  if (!bmp.begin(0x76))
  {
    Serial.println("BMP280 no encontrado!");
    while (1)
      delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

  compass.init();

  GPS_Serial.begin(9600);

}

void readMPU(float &x_accel, float &y_accel, float &z_accel, float &x_gyro, float &y_gyro, float &z_gyro)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  x_accel = a.acceleration.x;
  y_accel = a.acceleration.y;
  z_accel = a.acceleration.z;
  x_gyro = g.gyro.x;
  y_gyro = g.gyro.y;
  z_gyro = g.gyro.z;
}

void readBMP280(float &temperature, float &pressure, float &altitude)
{
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1013.25);
}

void calibrateAndReadCompass(int &x, int &y, int &z, int &azimuth, float &bearing)
{
  static unsigned long lastCalibrationTime = 0;
  bool changed = false;

  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  if (!calibrated)
  {
    int *axes[3] = {&x, &y, &z};
    for (int i = 0; i < 3; i++)
    {
      if (*axes[i] < calibrationData[i][0])
      {
        calibrationData[i][0] = *axes[i];
        changed = true;
      }
      if (*axes[i] > calibrationData[i][1])
      {
        calibrationData[i][1] = *axes[i];
        changed = true;
      }
    }

    if (changed)
    {
      lastCalibrationTime = millis();
    }

    if (millis() - lastCalibrationTime > 5000 && !calibrated)
    {
      calibrated = true;
      compass.setCalibration(calibrationData[0][0], calibrationData[0][1], calibrationData[1][0], calibrationData[1][1], calibrationData[2][0], calibrationData[2][1]);
      Serial.println("Calibración del compás completada.");
    }
  }

  azimuth = compass.getAzimuth();
  bearing = compass.getBearing(azimuth);
}

void retardoInteligente(unsigned long ms)
{
  unsigned long inicio = millis();
  do
  {
    while (GPS_Serial.available())
      gps.encode(GPS_Serial.read());
  } while (millis() - inicio < ms);
}

void DegMinSec(double tot_val)
{
  grados = (int)tot_val;
  minutos = tot_val - grados;
  segundos = 60 * minutos;
  mins = (int)minutos;
  minutos = minutos - mins;
  minutos = 60 * minutos;
  secs = (int)minutos;
}

void obtenerDatosGPS(double &latitud, double &longitud, double &altitud_m, uint8_t &hr, uint8_t &min, uint8_t &sec) {
  retardoInteligente(1000);
  latitud = gps.location.lat();
  longitud = gps.location.lng();
  altitud_m = gps.altitude.meters();
  hr = gps.time.hour();
  min = gps.time.minute();
  sec = gps.time.second();

  hr = (hr + 19) % 24; // Ajuste de zona horaria, modificar según necesidad
}


void loop()
{
  float x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro;
  readMPU(x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro);

  // Imprimir datos del MPU
  Serial.print("Aceleración - X: "); Serial.print(x_accel);
  Serial.print(", Y: "); Serial.print(y_accel);
  Serial.print(", Z: "); Serial.println(z_accel);
  Serial.print("Giroscopio - X: "); Serial.print(x_gyro);
  Serial.print(", Y: "); Serial.print(y_gyro);
  Serial.print(", Z: "); Serial.println(z_gyro);

  float temperature, pressure, altitude;
  readBMP280(temperature, pressure, altitude);

  // Imprimir datos del BMP280
  Serial.print("Temperatura: "); Serial.print(temperature); Serial.println(" C");
  Serial.print("Presión: "); Serial.print(pressure); Serial.println(" Pa");
  Serial.print("Altitud: "); Serial.print(altitude); Serial.println(" m");

  int x, y, z, azimuth;
  float bearing;
  calibrateAndReadCompass(x, y, z, azimuth, bearing);

  // Imprimir datos de la brújula
  Serial.print("Brújula - X: "); Serial.print(x);
  Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Z: "); Serial.print(z);
  Serial.print(", Azimuth: "); Serial.print(azimuth);
  Serial.print(", Bearing: "); Serial.println(bearing);

  double latitud, longitud, altitud_m;
  uint8_t hr, min, sec;
  obtenerDatosGPS(latitud, longitud, altitud_m, hr, min, sec);

  // Imprimir datos GPS
  Serial.print("Datos GPS: Latitud = "); Serial.print(latitud, 6);
  Serial.print(", Longitud = "); Serial.print(longitud, 6);
  Serial.print(", Altitud = "); Serial.print(altitud_m, 2); Serial.print(" m");
  Serial.print(", Hora = "); Serial.print(hr); Serial.print(":"); Serial.print(min); Serial.print(":"); Serial.println(sec);

  Serial.println("");
  Serial.println("");
  Serial.println("");

  delay(2000);
}

