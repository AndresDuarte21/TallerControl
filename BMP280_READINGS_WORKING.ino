#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

void setup() {
    Serial.begin(115200);
    if (!bmp.begin(0x76)) {   // Cambia a 0x77 si es necesario
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }

    // Configuración adicional (opcional)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Modo de operación */
                    Adafruit_BMP280::SAMPLING_X2,     /* Oversampling de temperatura */
                    Adafruit_BMP280::SAMPLING_X16,    /* Oversampling de presión */
                    Adafruit_BMP280::FILTER_X16,      /* Filtrado */
                    Adafruit_BMP280::STANDBY_MS_500); /* Tiempo de espera */
}

void loop() {
    Serial.print("Temperatura = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Presion = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Altitud = ");
    Serial.println(bmp.readAltitude(1013.25)); /* Ajuste el valor a la presión atmosférica local en hPa */
    
    delay(2000);
}
