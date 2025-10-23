#include <Arduino.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif

// #include <Wire.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535;
#define ICM_CS 10
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("Adafruit ICM20948 test!");
  Wire.setPins(9, 8);
  // if (!icm.begin_I2C(0x68, &Wire))
  if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI))
  {
    Serial.println("Failed to find ICM20948 chip");
    pinMode(LED_BUILTIN, OUTPUT);
    uint tdelay = 100;
    uint d = 0;
    float freq = 0.5;
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(tdelay/10);
      digitalWrite(LED_BUILTIN, LOW);
      delay(tdelay);
      // d += 1;
      // uint brightness = int(sinf(float(millis())/1000.0*2.0*PI * freq)*127.0 + 128.0);
      // // analogWrite(LED_BUILTIN, sin(d / tdelay * 1 / 2 / PI) * 127 + 128);
      // analogWrite(LED_BUILTIN, brightness);
      // Serial.println(brightness);

      delay(tdelay);
    }
  }
  Serial.println("ICM20948 Found!");
  icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  if (icm.getAccelRange() != ICM20948_ACCEL_RANGE_8_G)
  {
    Serial.println("Failed to set accelerometer range");
    while (1)
    {
      delay(100);
    }
  }
  Serial.print("Accelerometer range set to 8 G");

  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  if (icm.getGyroRange() != ICM20948_GYRO_RANGE_2000_DPS)
  {
    Serial.println("Failed to set gyro range");
    while (1)
    {
      delay(100);
    }
  }
  Serial.print("Gyro range set to 2000 deg/s");

  icm.setAccelRateDivisor(0);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

   icm.setGyroRateDivisor(0);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  delay(100);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  Serial.println(icm.getMagDataRate());
  if (icm.getMagDataRate() != AK09916_MAG_DATARATE_100_HZ)
  {
    Serial.println("Failed to set magnetometer data rate");
    while (1)
    {
      delay(100);
    }
  }
  Serial.print("Magnetometer data rate set to 100 Hz");
  Serial.println();
}

void loop()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Serial.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
  //               gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
  //               mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tMag X: ");
  Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z);
  Serial.println(" uT");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(1000);
}
