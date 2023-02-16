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
  Serial.println("Two Sensors Test");

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
}

void loop(void)
{
  distanceSensor1.startRanging();
  distanceSensor2.startRanging();
  while (!distanceSensor1.checkForDataReady())
  {
    delay(1);
  }
  int distance1 = distanceSensor1.getDistance();
  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();
  while (!distanceSensor2.checkForDataReady())
  {
    delay(1);
  }
  int distance2 = distanceSensor2.getDistance();
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();

  Serial.print("Distance(mm): ");
  Serial.print(distance1);

  float distance1Inches = distance1 * 0.0393701;
  float distance1Feet = distance1Inches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distance1Feet, 2);

  Serial.print("\t|\tDistance(mm): ");
  Serial.print(distance2);

  float distance2Inches = distance2 * 0.0393701;
  float distance2Feet = distance2Inches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distance2Feet, 2);

  Serial.println();
}