// Basic demo for pressure readings from Adafruit LPS2X
#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>

// For SPI mode, we need a CS pin
#define LPS_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LPS_SCK 13
#define LPS_MISO 12
#define LPS_MOSI 11

Adafruit_LPS22 lps;
Adafruit_LPS22 lps2;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LPS22 test!");

  // Try to initialize!
  if (!lps.begin_I2C(0x5C)) {
    Serial.println("Failed to find LPS22 chip");
    while (1) { delay(10); }
  }
 if (!lps2.begin_I2C(0x5D)) {
    Serial.println("Failed to find LPS22-2 chip");
    while (1) { delay(10); }
  }
  Serial.println("Both LPS22 Found!");

  lps.setDataRate(LPS22_RATE_10_HZ);
  lps2.setDataRate(LPS22_RATE_10_HZ);
  Serial.print("Data rate set to: ");
  switch (lps.getDataRate()) {
    case LPS22_RATE_ONE_SHOT: Serial.println("One Shot / Power Down"); break;
    case LPS22_RATE_1_HZ: Serial.println("1 Hz"); break;
    case LPS22_RATE_10_HZ: Serial.println("10 Hz"); break;
    case LPS22_RATE_25_HZ: Serial.println("25 Hz"); break;
    case LPS22_RATE_50_HZ: Serial.println("50 Hz"); break;

  }
}

void loop() {
  sensors_event_t temp;
  sensors_event_t pressure;
  sensors_event_t temp2;
  sensors_event_t pressure2;
  lps.getEvent(&pressure, &temp);// get pressure
  Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.print(" degrees C,");
  Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.print(" hPa,");
  lps2.getEvent(&pressure2, &temp2);// get pressure
  Serial.print("Temperature2: ");Serial.print(temp2.temperature);Serial.print(" degrees C,");
  Serial.print("Pressure2: ");Serial.print(pressure2.pressure);Serial.println(" hPa");
  Serial.println("");
  delay(100);
}
