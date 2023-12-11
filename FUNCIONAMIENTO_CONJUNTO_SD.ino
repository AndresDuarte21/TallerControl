#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>


Servo myServo;


Adafruit_BMP280 bmp; // I2C
QMC5883LCompass compass;

int calibrationData[3][2] = {{INT_MAX, INT_MIN}, {INT_MAX, INT_MIN}, {INT_MAX, INT_MIN}};
bool calibrated = false;

SoftwareSerial GPS_Serial(4, 5); // RX, TX
TinyGPSPlus gps;

volatile float minutos, segundos;
volatile int grados, secs, mins;

const int chipSelect = 10;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    delay(10); // Esperar conexión serial


  //Inicialización del Servo
  myServo.attach(3); // D3
  myServo.write(0);


  if (!bmp.begin(0x76))
  {
    Serial.println("BMP280 no encontrado!");
    while (1)
      delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

  compass.init();

  GPS_Serial.begin(9600);
  
  // Inicialización de la tarjeta SD
  if (!SD.begin(chipSelect))
  {
    Serial.println("Error al inicializar la tarjeta SD.");
    return;
  }
  Serial.println("Tarjeta SD inicializada correctamente.");

  // Crear archivo CSV si no existe
  File archivo = SD.open("/datos.csv", FILE_WRITE);
  if (archivo)
  {
    Serial.println("Archivo 'datos.csv' listo para escritura.");
    // Escribir cabeceras de columnas en el archivo CSV
    archivo.println("Tiempo, Acelerometro_X, Acelerometro_Y, Acelerometro_Z, Giroscopio_X, Giroscopio_Y, Giroscopio_Z, Temperatura, Presion, Altitud, Compas_X, Compas_Y, Compas_Z, Azimut, Bearing, GPS_Lat, GPS_Lng, GPS_Alt, GPS_Hora");
    archivo.close();
  }
  else
  {
    Serial.println("Error al abrir el archivo 'datos.csv'.");
  }
}

void releaseParachute() {
  myServo.write(180);  // Cambia la posición del servo a 180 grados
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
  retardoInteligente(3000);
  latitud = gps.location.lat();
  longitud = gps.location.lng();
  altitud_m = gps.altitude.meters();
  hr = gps.time.hour();
  min = gps.time.minute();
  sec = gps.time.second();

  hr = (hr + 19) % 24; // Ajuste de zona horaria, modificar según necesidad
}


void guardarDatosCSV(unsigned long tiempo, float temperature, float pressure, float altitude, int x, int y, int z, int azimuth, float bearing, double latitud, double longitud, double altitud_m, uint8_t hr, uint8_t min, uint8_t sec) {
  File archivo = SD.open("/datos.csv", FILE_WRITE);
  if (archivo) {
    // Escribe los datos en formato CSV
    archivo.print(tiempo); archivo.print(", ");
    archivo.print(temperature); archivo.print(", ");
    archivo.print(pressure); archivo.print(", ");
    archivo.print(altitude); archivo.print(", ");
    archivo.print(x); archivo.print(", ");
    archivo.print(y); archivo.print(", ");
    archivo.print(z); archivo.print(", ");
    archivo.print(azimuth); archivo.print(", ");
    archivo.print(bearing); archivo.print(", ");
    archivo.print(latitud, 6); archivo.print(", ");
    archivo.print(longitud, 6); archivo.print(", ");
    archivo.print(altitud_m, 2); archivo.print(", ");
    archivo.print(hr); archivo.print(":");
    archivo.print(min); archivo.print(":");
    archivo.println(sec);
    Serial.println("Guardado exitoso.");
    archivo.close();
  } else {
    Serial.println("Error al abrir 'datos.csv' para escritura.");
  }
}


void loop()
{


  float temperature, pressure, altitude;
  readBMP280(temperature, pressure, altitude);

  int x, y, z, azimuth;
  float bearing;
  calibrateAndReadCompass(x, y, z, azimuth, bearing);


  double latitud, longitud, altitud_m;
  uint8_t hr, min, sec;
  obtenerDatosGPS(latitud, longitud, altitud_m, hr, min, sec);
  Serial.print("Datos GPS: Latitud = "); Serial.print(latitud, 6);
  Serial.print(", Longitud = "); Serial.print(longitud, 6);
  Serial.print(", Altitud = "); Serial.print(altitud_m, 2); Serial.print(" m");
  Serial.print(", Hora = "); Serial.print(hr); Serial.print(":"); Serial.print(min); Serial.print(":"); Serial.println(sec);

  unsigned long tiempo = millis(); // Obtener el tiempo actual
  guardarDatosCSV(tiempo, temperature, pressure, altitude, x, y, z, azimuth, bearing, latitud, longitud, altitud_m, hr, min, sec);

  delay(2000);
}