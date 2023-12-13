#include <Arduino.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

int calibrationData[3][2];
bool calibrated = false;

void calibrateCompass() {
  static bool changed = false;
  static bool done = false;
  static int t = 0;
  static int c = 0;

  int x, y, z;
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  changed = false;

  for (int i = 0; i < 3; i++) {
    if (x < calibrationData[i][0]) {
      calibrationData[i][0] = x;
      changed = true;
    }
    if (x > calibrationData[i][1]) {
      calibrationData[i][1] = x;
      changed = true;
    }
  }

  for (int i = 0; i < 3; i++) {
    if (y < calibrationData[i][0]) {
      calibrationData[i][0] = y;
      changed = true;
    }
    if (y > calibrationData[i][1]) {
      calibrationData[i][1] = y;
      changed = true;
    }
  }

  for (int i = 0; i < 3; i++) {
    if (z < calibrationData[i][0]) {
      calibrationData[i][0] = z;
      changed = true;
    }
    if (z > calibrationData[i][1]) {
      calibrationData[i][1] = z;
      changed = true;
    }
  }

  if (changed && !done) {
    Serial.println("CALIBRATING... Keep moving your sensor around.");
    c = millis();
  }
  t = millis();

  if ((t - c > 5000) && !done) {
    done = true;
    Serial.println("DONE.");
    Serial.println();

    Serial.print("compass.setCalibration(");
    for (int i = 0; i < 3; i++) {
      Serial.print(calibrationData[i][0]);
      Serial.print(", ");
      Serial.print(calibrationData[i][1]);
      if (i < 2) {
        Serial.print(", ");
      }
    }
    Serial.println(");");

    compass.setCalibration(calibrationData[0][0], calibrationData[0][1], calibrationData[1][0],
                           calibrationData[1][1], calibrationData[2][0], calibrationData[2][1]);
    calibrated = true;
  }
}

void readAndPrintCompass() {
  int x, y, z;
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  int azimuth = compass.getAzimuth();
  float bearing = compass.getBearing(azimuth);

  Serial.println();

  if ((azimuth < 22.5) || (azimuth > 337.5)) Serial.print("North     ");
  if ((azimuth > 22.5) && (azimuth < 67.5)) Serial.print("North-East");
  if ((azimuth > 67.5) && (azimuth < 112.5)) Serial.print("East      ");
  if ((azimuth > 112.5) && (azimuth < 157.5)) Serial.print("South-East");
  if ((azimuth > 157.5) && (azimuth < 202.5)) Serial.print("South     ");
  if ((azimuth > 202.5) && (azimuth < 247.5)) Serial.print("South-West");
  if ((azimuth > 247.5) && (azimuth < 292.5)) Serial.print("West      ");
  if ((azimuth > 292.5) && (azimuth < 337.5)) Serial.print("North-West");

  Serial.print(" Azimuth: ");
  Serial.print(azimuth);
  Serial.print(" Bearing: ");
  Serial.print(bearing);

  Serial.print(" - X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);

  delay(250);
}

void setup() {
  Serial.begin(9600);
  compass.init();
}

void loop() {
  if (!calibrated) {
    calibrateCompass();
  } else {
    readAndPrintCompass();
  }
}

