#define MAX_MSG_SIZE 151

#include <math.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_VL53L1X.h>
#include "BLECStringCharacteristic.h"
#include "BLETxStream.h"
#include "RobotCommand.h"

enum CommandTypes {
  RUN_MAP,
  DATA_MAP,
  PID_MAP,
  JUST_MAP,
};

#pragma region BLE_UUIDs
//////////// BLE UUIDs ////////////

#define BLE_UUID_MY_SERVICE "2ed7b712-660d-4e3d-aa5e-812641c1dfdd"

#define BLE_UUID_RX_CMD "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_RX_PWM "911532ae-004d-4c02-9813-7e494efd5408"

#define BLE_UUID_TX_STREAM "51cc72fd-5d0a-48f9-9d9f-7829f37e1f02"

//////////// BLE UUIDs ////////////
#pragma endregion BLE_UUIDs

#pragma region Sensor_Config
//////////// Sensor Config ////////////

#define SENSOR1_ADDR 0x32
#define SENSOR2_SHUT 8

//////////// Sensor Config ////////////
#pragma endregion Sensor_Config

#pragma region Motor_Config
//////////// Motor Config ////////////

#define MOTOR1_FWD A3
#define MOTOR1_BCK A2

#define MOTOR2_FWD A15
#define MOTOR2_BCK A14

//////////// Motor Config ////////////
#pragma endregion Motor_Config

#pragma region BLE
//////////// BLE ////////////

BLEService myService(BLE_UUID_MY_SERVICE);

BLECStringCharacteristic rx_characteristic_cmd(BLE_UUID_RX_CMD, BLEWrite, MAX_MSG_SIZE);
RobotCommand robot_cmd(":|");
BLEUnsignedLongCharacteristic rx_characteristic_pwm(BLE_UUID_RX_PWM, BLEWrite);

BLECharacteristic tx_characteristic_stream(BLE_UUID_TX_STREAM, BLERead | BLENotify, MAX_MSG_SIZE);
BLETxStream tx_stream(&tx_characteristic_stream);

//////////// BLE ////////////
#pragma endregion BLE

#pragma region Sensors
//////////// Sensors ////////////

ICM_20948_I2C myICM;

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;

//////////// Sensors ////////////
#pragma endregion Sensors

#pragma region Buffers
//////////// Buffers ////////////

constexpr size_t IMU_BUF_LEN = 960;
float imu_buf[7 * IMU_BUF_LEN];
size_t imu_buf_len = 0;

constexpr size_t TOF1_BUF_LEN = 160;
unsigned long tof1_time_buf[TOF1_BUF_LEN];
uint16_t tof1_data_buf[TOF1_BUF_LEN];
size_t tof1_buf_len = 0;

constexpr size_t TOF2_BUF_LEN = 160;
unsigned long tof2_time_buf[TOF2_BUF_LEN];
uint16_t tof2_data_buf[TOF2_BUF_LEN];
size_t tof2_buf_len = 0;

//////////// Buffers ////////////
#pragma endregion Buffers

void motorWrite(uint8_t leftFwd, uint8_t leftBck, uint8_t rightFwd, uint8_t rightBck) {
  analogWrite(MOTOR1_FWD, leftFwd);
  analogWrite(MOTOR1_BCK, leftBck);
  analogWrite(MOTOR2_FWD, rightFwd);
  analogWrite(MOTOR2_BCK, rightBck);
}
void motorWrite(int16_t left, int16_t right) {
  if (left >= 0) {
    analogWrite(MOTOR1_FWD, left);
    analogWrite(MOTOR1_BCK, 0);
  } else {
    analogWrite(MOTOR1_FWD, 0);
    analogWrite(MOTOR1_BCK, left);
  }
  if (right >= 0) {
    analogWrite(MOTOR2_FWD, right);
    analogWrite(MOTOR2_BCK, 0);
  } else {
    analogWrite(MOTOR2_FWD, 0);
    analogWrite(MOTOR2_BCK, right);
  }
}

void pwmCharacteristicWritten(BLEDevice central, BLECharacteristic rx_characteristic) {
  const uint8_t* bytes = rx_characteristic.value();
  motorWrite(bytes[0], bytes[1], bytes[2], bytes[3]);
  Serial.print("PWM: ");
  Serial.println(*(const unsigned long*) bytes, HEX);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();

  // BLE setup
  BLE.begin();
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(myService);
  myService.addCharacteristic(rx_characteristic_cmd);
  myService.addCharacteristic(rx_characteristic_pwm);
  myService.addCharacteristic(tx_characteristic_stream);
  BLE.addService(myService);
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());
  BLE.advertise();

  // Distance Sensor setup
  pinMode(SENSOR2_SHUT, OUTPUT);
  digitalWrite(SENSOR2_SHUT, LOW);
  if (distanceSensor1.begin() != 0) {
    Serial.println("Sensor 1 failed to begin");
    while (true);
  }
  // distanceSensor1.setI2CAddress(SENSOR1_ADDR);
  Serial.println("Sensor 1 online");
  // digitalWrite(SENSOR2_SHUT, HIGH);
  // if (distanceSensor2.begin() != 0) {
  //   Serial.println("Sensor 2 failed to begin");
  //   while (true);
  // }
  // Serial.println("Sensor 2 online");
  myICM.begin();
  if (myICM.status != 0) {
    Serial.println(myICM.statusString());
    Serial.println("IMU failed to begin");
    while (true);
  }
  Serial.println("IMU online");

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

float calibrate_gyro(unsigned long duration) {
  int count = 0;
  float acc = 0;
  unsigned long startMillis = millis();
  while (millis() - startMillis < duration) {
    myICM.getAGMT();
    acc += myICM.gyrZ();
    ++count;
  }
  return acc / count;
}

void run_map(bool logPid, unsigned long calibrateDuration, float maxAngle, float deltaAngle, float tol, float minRange, float maxRange, uint8_t minPower, uint8_t maxPower, float k_p, float k_i, float k_d) {
  float calibration = calibrate_gyro(calibrateDuration);

  unsigned long previousMillis, currentMillis;
  float currentAngle = 0;
  while (abs(currentAngle) <= abs(maxAngle)) {
    // Take a measurement
    distanceSensor1.startOneshotRanging();
    while (!distanceSensor1.checkForDataReady())
      delay(1);
    currentMillis = millis();
    uint16_t distance = distanceSensor1.getDistance();

    tof1_time_buf[tof1_buf_len] = currentMillis;
    tof2_time_buf[tof1_buf_len] = *(unsigned long*) &currentAngle;
    tof1_data_buf[tof1_buf_len] = distance;
    ++tof1_buf_len;

    // Rotate
    float targetAngle = currentAngle + deltaAngle;
    float integrator = 0;
    previousMillis = currentMillis;
    myICM.getAGMT();
    while (abs(currentAngle - targetAngle) > tol) {
      float dt = (currentMillis - previousMillis) / 1000.0;
      float speed = myICM.gyrZ() - calibration;
      currentAngle += speed * dt;

      float error = currentAngle - targetAngle;
      integrator += error * dt;
      float pid = k_p * error + k_i * integrator + k_d * speed;

      uint8_t motor = (abs(pid) - minRange) * (maxPower - minPower) / (maxRange - minRange) + minPower;
      if (pid < 0) {
        motorWrite(motor, 0, 0, motor);
      } else {
        motorWrite(0, motor, motor, 0);
      }

      if (logPid) {
        uint32_t motor_pad = motor;
        imu_buf[imu_buf_len++] = *(float*) &currentMillis;
        imu_buf[imu_buf_len++] = speed;
        imu_buf[imu_buf_len++] = currentAngle;
        imu_buf[imu_buf_len++] = error;
        imu_buf[imu_buf_len++] = integrator;
        imu_buf[imu_buf_len++] = pid;
        imu_buf[imu_buf_len++] = motor_pad;
      }

      previousMillis = currentMillis;
      myICM.getAGMT();
      currentMillis = millis();
    }

    motorWrite(255, 255, 255, 255);
  }
  motorWrite(0, 0, 0, 0);
}

void handle_command() {
  robot_cmd.set_cmd_string(
    rx_characteristic_cmd.value(),
    rx_characteristic_cmd.valueLength()
  );
  int cmd_type = -1;
  bool success = robot_cmd.get_command_type(cmd_type);
  if (!success) return;

  unsigned long startMillis, currentMillis;
  int duration;
  int ival1, ival2, ival3, ival4;
  float fval1, fval2, fval3, fval4, fval5, fval6, fval7, fval8;
  switch (cmd_type) {

    /**
      * RUN_MAP:calibrateDuration|maxAngle|deltaAngle|tolerance|minPid|maxPid|minMotor|maxMotor|k_p|k_i|k_d
      * 
      */
    case RUN_MAP:
      success = robot_cmd.get_next_value(duration)
        && robot_cmd.get_next_value(fval1)
        && robot_cmd.get_next_value(fval2)
        && robot_cmd.get_next_value(fval3)
        && robot_cmd.get_next_value(fval4)
        && robot_cmd.get_next_value(fval5)
        && robot_cmd.get_next_value(ival1)
        && robot_cmd.get_next_value(ival2)
        && robot_cmd.get_next_value(fval6)
        && robot_cmd.get_next_value(fval7)
        && robot_cmd.get_next_value(fval8)
      ;
      if (!success) return;
      run_map(true, duration, fval1, fval2, fval3, fval4, fval5, ival1, ival2, fval6, fval7, fval8);
      break;

    /**
      * JUST_MAP:calibrateDuration|maxAngle|deltaAngle|tolerance|minPid|maxPid|minMotor|maxMotor|k_p|k_i|k_d
      * 
      */
    case JUST_MAP:
      success = robot_cmd.get_next_value(duration)
        && robot_cmd.get_next_value(fval1)
        && robot_cmd.get_next_value(fval2)
        && robot_cmd.get_next_value(fval3)
        && robot_cmd.get_next_value(fval4)
        && robot_cmd.get_next_value(fval5)
        && robot_cmd.get_next_value(ival1)
        && robot_cmd.get_next_value(ival2)
        && robot_cmd.get_next_value(fval6)
        && robot_cmd.get_next_value(fval7)
        && robot_cmd.get_next_value(fval8)
      ;
      if (!success) return;
      run_map(false, duration, fval1, fval2, fval3, fval4, fval5, ival1, ival2, fval6, fval7, fval8);
      break;
    
    /**
      * DATA_MAP:
      * [time{u4},angle{f4},distance{u2}]*
      */
    case DATA_MAP:
      for (size_t i = 0; i < tof1_buf_len; ++i) {
        tx_stream.write(tof1_time_buf[i]);
        tx_stream.write(tof2_time_buf[i]);
        tx_stream.write(tof1_data_buf[i]);
      }
      tof1_buf_len = 0;
      tx_stream.flush();
      break;
    
    /**
      * PID_MAP:
      * [time{u4},speed{f4},angle{f4},error{f4},integrator{f4},pid{f4},motor{u4}]*
      */
    case PID_MAP:
      tx_stream.write(imu_buf, imu_buf_len);
      imu_buf_len = 0;
      tx_stream.flush();
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
    motorWrite(0, 0);
    Serial.println("Disconnected");
  }
}
