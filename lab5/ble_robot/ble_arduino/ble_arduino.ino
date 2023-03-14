#define MAX_MSG_SIZE 151

#include <ArduinoBLE.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_VL53L1X.h>
#include "BLECStringCharacteristic.h"
#include "RobotCommand.h"

enum CommandTypes {
  DRIVE,
  OPEN_LOOP,
};

#pragma region BLE_UUIDs
//////////// BLE UUIDs ////////////

#define BLE_UUID_MY_SERVICE "2ed7b712-660d-4e3d-aa5e-812641c1dfdd"

#define BLE_UUID_RX_CMD "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_RX_PWM "911532ae-004d-4c02-9813-7e494efd5408"

//////////// BLE UUIDs ////////////
#pragma endregion BLE_UUIDs

#pragma region Sensor_Config
//////////// Sensor Config ////////////

#define IMU_AD0_VAL 1

#define SENSOR1_ADDR 0x32
#define SENSOR2_SHUT 8

//////////// Sensor Config ////////////
#pragma endregion Sensor_Config

#pragma region Motor_Config
//////////// Motor Config ////////////

#define MOTOR1_FWD A2
#define MOTOR1_BCK A3

#define MOTOR2_FWD A14
#define MOTOR2_BCK A15

//////////// Motor Config ////////////
#pragma endregion Motor_Config

#pragma region BLE
//////////// BLE ////////////

BLEService myService(BLE_UUID_MY_SERVICE);

BLECStringCharacteristic rx_characteristic_cmd(BLE_UUID_RX_CMD, BLEWrite, MAX_MSG_SIZE);
RobotCommand robot_cmd(":|");
BLEUnsignedLongCharacteristic rx_characteristic_pwm(BLE_UUID_RX_PWM, BLEWrite);

//////////// BLE ////////////
#pragma endregion BLE

#pragma region Sensors
//////////// Sensors ////////////

ICM_20948_I2C myICM;

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;

//////////// Sensors ////////////
#pragma endregion Sensors

void pwmCharacteristicWritten(BLEDevice central, BLECharacteristic rx_characteristic) {
  const uint8_t* bytes = rx_characteristic.value();
  analogWrite(MOTOR1_FWD, bytes[0]);
  analogWrite(MOTOR1_BCK, bytes[1]);
  analogWrite(MOTOR2_FWD, bytes[2]);
  analogWrite(MOTOR2_BCK, bytes[3]);
  Serial.print("PWM: ");
  Serial.println(*(const unsigned long*) bytes, HEX);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // BLE setup
  BLE.begin();
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(myService);
  myService.addCharacteristic(rx_characteristic_cmd);
  myService.addCharacteristic(rx_characteristic_pwm);
  BLE.addService(myService);
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());
  BLE.advertise();

  // Sensor setup

  // Motor setup
  pinMode(MOTOR1_FWD, OUTPUT);
  pinMode(MOTOR1_BCK, OUTPUT);
  pinMode(MOTOR2_FWD, OUTPUT);
  pinMode(MOTOR2_BCK, OUTPUT);
  rx_characteristic_pwm.setEventHandler(BLEWritten, pwmCharacteristicWritten);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void handle_command() {
  robot_cmd.set_cmd_string(
    rx_characteristic_cmd.value(),
    rx_characteristic_cmd.valueLength()
  );
  int cmd_type = -1;
  bool success = robot_cmd.get_command_type(cmd_type);
  if (!success) return;

  int duration;
  int ival1, ival2, ival3, ival4;
  switch (cmd_type) {

    /**
      * DRIVE:duration|motor1_fwd|motor1_bck|motor2_fwd|motor2_bck
      * Drive (blocking) for the duration (in milliseconds) with the given motor PWM values ([0..255]).
      */
    case DRIVE:
      success = robot_cmd.get_next_value(duration)
        && robot_cmd.get_next_value(ival1)
        && robot_cmd.get_next_value(ival2)
        && robot_cmd.get_next_value(ival3)
        && robot_cmd.get_next_value(ival4)
      ;
      if (!success) return;
      analogWrite(MOTOR1_FWD, ival1);
      analogWrite(MOTOR1_BCK, ival2);
      analogWrite(MOTOR2_FWD, ival3);
      analogWrite(MOTOR2_BCK, ival4);
      delay(duration);
      analogWrite(MOTOR1_FWD, 0);
      analogWrite(MOTOR1_BCK, 0);
      analogWrite(MOTOR2_FWD, 0);
      analogWrite(MOTOR2_BCK, 0);
      break;
    
    /**
      * OPEN_LOOP:
      * Drive (blocking) the open loop path
      */
    case OPEN_LOOP:
      analogWrite(MOTOR1_FWD, 138);
      analogWrite(MOTOR1_BCK, 1);
      analogWrite(MOTOR2_FWD, 100);
      analogWrite(MOTOR2_BCK, 1);
      delay(600);
      analogWrite(MOTOR1_FWD, 1);
      analogWrite(MOTOR1_BCK, 238);
      analogWrite(MOTOR2_FWD, 200);
      analogWrite(MOTOR2_BCK, 1);
      delay(250);
      analogWrite(MOTOR1_FWD, 138);
      analogWrite(MOTOR1_BCK, 1);
      analogWrite(MOTOR2_FWD, 100);
      analogWrite(MOTOR2_BCK, 1);
      delay(400);
      analogWrite(MOTOR1_FWD, 238);
      analogWrite(MOTOR1_BCK, 1);
      analogWrite(MOTOR2_FWD, 1);
      analogWrite(MOTOR2_BCK, 200);
      delay(250);
      analogWrite(MOTOR1_FWD, 138);
      analogWrite(MOTOR1_BCK, 1);
      analogWrite(MOTOR2_FWD, 100);
      analogWrite(MOTOR2_BCK, 1);
      delay(600);
      analogWrite(MOTOR1_FWD, 0);
      analogWrite(MOTOR1_BCK, 0);
      analogWrite(MOTOR2_FWD, 0);
      analogWrite(MOTOR2_BCK, 0);
      break;

    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    while (central.connected()) {
      if (rx_characteristic_cmd.written())
        handle_command();
    }
    Serial.println("Disconnected");
  }
}
