#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial GPS_Serial(4, 5); // RX, TX
TinyGPSPlus gps;

volatile float minutos, segundos;
volatile int grados, secs, mins;

void setup() {
  Serial.begin(9600);
  GPS_Serial.begin(9600);
}

static void retardoInteligente(unsigned long ms) {
  unsigned long inicio = millis();
  do {
    while (GPS_Serial.available())
      gps.encode(GPS_Serial.read());
  } while (millis() - inicio < ms);
}

void DegMinSec(double tot_val) {
  grados = (int)tot_val;
  minutos = tot_val - grados;
  segundos = 60 * minutos;
  mins = (int)minutos;
  minutos = minutos - mins;
  minutos = 60 * minutos;
  secs = (int)minutos;
}

String obtenerDatosGPS() {
  retardoInteligente(1000);
  unsigned long inicio;
  double latitud, longitud, altitud_m;
  uint8_t hr, min, sec;
  bool loc_valida, alt_valida, hora_valida;

  latitud = gps.location.lat();
  loc_valida = gps.location.isValid();
  longitud = gps.location.lng();
  altitud_m = gps.altitude.meters();
  alt_valida = gps.altitude.isValid();
  hr = gps.time.hour();
  min = gps.time.minute();
  sec = gps.time.second();
  hora_valida = gps.time.isValid();

  hr = (hr + 19) % 24;

  String resultado = "";

  if (!loc_valida) {
    resultado += "latitud = ***";
  } else {
    DegMinSec(latitud);
    resultado += "latitud = " + String(latitud, 6);
  }

  if (!loc_valida || !alt_valida) {
    resultado += ", ";
  } else {
    resultado += ", ";
  }

  if (!loc_valida || !alt_valida) {
    resultado += "longitud = ***";
  } else {
    DegMinSec(longitud);
    resultado += "longitud = " + String(longitud, 6);
  }

  if (!alt_valida) {
    resultado += ", ";
  } else {
    resultado += ", ";
    resultado += "altitud = " + String(altitud_m, 6) + " metros";
  }

  if (!hora_valida) {
    resultado += "hora = ***";
  } else {
    resultado += " hora = " + String(hr) + ":" + String(min) + ":" + String(sec);
  }

  return resultado;
}

void loop() {
  String datosGPS = obtenerDatosGPS();
  Serial.println(datosGPS);
}