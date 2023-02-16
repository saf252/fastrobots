#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SENSOR1_ADDR 0x32
#define SENSOR2_SHUT D8

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("Two Sensors Rapid Test");

  pinMode(SENSOR2_SHUT, OUTPUT);

  // disable sensor 2 to initialize sensor 1
  digitalWrite(SENSOR2_SHUT, LOW);
  distanceSensor1.setI2CAddress(SENSOR1_ADDR);
  if (distanceSensor1.begin() != 0)
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 1 online!");
  distanceSensor1.setI2CAddress(SENSOR1_ADDR);

  // initialize sensor 2
  digitalWrite(SENSOR2_SHUT, HIGH);
  if (distanceSensor2.begin() != 0)
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 2 online!");
  distanceSensor2.setDistanceModeShort();

  distanceSensor1.startRanging();
  distanceSensor2.startRanging();
}

void loop(void)
{
  Serial.print(millis());

  if (distanceSensor1.checkForDataReady())
  {
    Serial.print("\tSensor 1 -> Distance (mm): ");
    Serial.print(distanceSensor1.getDistance());
    distanceSensor1.clearInterrupt();
  }
  if (distanceSensor2.checkForDataReady())
  {
    Serial.print("\tSensor 2 -> Distance (mm): ");
    Serial.print(distanceSensor2.getDistance());
    distanceSensor2.clearInterrupt();
  }

  Serial.println();
}