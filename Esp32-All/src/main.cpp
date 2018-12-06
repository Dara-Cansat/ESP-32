#include <Arduino.h>


#include <Wire.h>
#include <Adafruit_BMP280.h>
static const uint32_t GPSBaud = 9600;

#include <TinyGPS++.h>
TinyGPSPlus gps;
// HardwareSerial mySerial1(2);
void processGps();
void bme280print();
Adafruit_BMP280 bmp; // I2C

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 16 /*rx*/, 17/* tx */);
  

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
    processGps();
    bme280print();
    
}

void processGps() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
        RawDegrees lat = gps.location.rawLat();
        RawDegrees lng = gps.location.rawLng();
        char latC[20];
        char lngC[20];
        sprintf(latC, "%c%u.%lu", lat.negative?'-':' ', lat.deg, lat.billionths);
        sprintf(lngC, "%c%u.%lu", lng.negative?'-':' ', lng.deg, lng.billionths);
        Serial.printf("%s,%s\r\n", latC, lngC);
      }
      else
      {
        // Serial.println(F("INVALID"));
      }
    }
  }
}


void bme280print(){
   Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
    
    delay(2000);
}
