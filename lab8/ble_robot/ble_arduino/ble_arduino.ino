#define MAX_MSG_SIZE 151

#include <math.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_VL53L1X.h>
#include "BLECStringCharacteristic.h"
#include "BLETxStream.h"
#include "RobotCommand.h"
#include "PID.h"

enum CommandTypes {
  DRIVE,
  OPEN_LOOP,
  RUN_PID,
  DATA_PID,
  EXTRA_DATA_PID,
  STREAM_TOF1,
  RUN_STUNT,
  DATA_STUNT,
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

void pwmCharacteristicWritten(BLEDevice central, BLECharacteristic rx_characteristic) {
  const uint8_t* bytes = rx_characteristic.value();
  analogWrite(MOTOR1_FWD, bytes[0]);
  analogWrite(MOTOR1_BCK, bytes[1]);
  analogWrite(MOTOR2_FWD, bytes[2]);
  analogWrite(MOTOR2_BCK, bytes[3]);
  Serial.print("PWM: ");
  Serial.println(*(const unsigned long*) bytes, HEX);
}

void shutdown() {
  analogWrite(MOTOR1_FWD, 0);
  analogWrite(MOTOR1_BCK, 0);
  analogWrite(MOTOR2_FWD, 0);
  analogWrite(MOTOR2_BCK, 0);
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
  myService.addCharacteristic(tx_characteristic_stream);
  BLE.addService(myService);
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());
  BLE.advertise();

  // Sensor setup
  Wire.begin();
  pinMode(SENSOR2_SHUT, OUTPUT);
  digitalWrite(SENSOR2_SHUT, LOW);
  if (distanceSensor1.begin() != 0) {
    Serial.println("Sensor 1 failed to begin");
    while (true);
  }
  // distanceSensor1.setI2CAddress(SENSOR1_ADDR);
  Serial.println("Sensor 1 online");
  // digitalWrite(SENSOR2_SHUT, HIGH);
  // if (!distanceSensor2.begin()) {
  //   Serial.println("Sensor 2 failed to begin");
  //   while (true);
  // }
  // Serial.println("Sensor 2 online");
  // if (!myICM.begin()) {
  //   Serial.println("IMU failed to begin");
  //   while (true);
  // }
  // Serial.println("IMU online");

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

void run_pid(unsigned long timeout, unsigned short goal, uint8_t deadband, float calibration, float k_p, float k_i, float k_d) {
  PID params = { (float) goal, k_p, k_i, k_d };
  unsigned long startMillis, currentMillis;
  distanceSensor1.startRanging();
  startMillis = millis();
  do {
    currentMillis = millis();
    if (distanceSensor1.checkForDataReady()) {
      uint16_t distance = distanceSensor1.getDistance();
      distanceSensor1.clearInterrupt();
      
      // Compute PID
      float raw = pid(currentMillis, distance, params, !tof1_buf_len);
      uint8_t left = constrain(abs(raw), deadband, 255);
      uint8_t right = constrain(abs(raw) * calibration, deadband, 255);

      // Scale PID for motors
      if (raw > 0) {
        analogWrite(MOTOR1_FWD, left);
        analogWrite(MOTOR2_FWD, right);
        analogWrite(MOTOR1_BCK, 0);
        analogWrite(MOTOR2_BCK, 0);
      } else {
        analogWrite(MOTOR1_FWD, 0);
        analogWrite(MOTOR2_FWD, 0);
        analogWrite(MOTOR1_BCK, left);
        analogWrite(MOTOR2_BCK, right);
      }

      // Store data in buffers, using tof2 time for raw PID and data for motor values
      tof1_time_buf[tof1_buf_len] = currentMillis;
      tof1_data_buf[tof1_buf_len] = distance;
      tof2_time_buf[tof1_buf_len] = *(unsigned long*) &raw;
      tof2_data_buf[tof1_buf_len] = makeWord(right, left);
      ++tof1_buf_len;
    }
  } while (currentMillis - startMillis < timeout);
  analogWrite(MOTOR1_FWD, 0);
  analogWrite(MOTOR2_FWD, 0);
  analogWrite(MOTOR1_BCK, 0);
  analogWrite(MOTOR2_BCK, 0);
  distanceSensor1.stopRanging();
}

void run_stunt(unsigned long timeout, unsigned short target, uint8_t power_l, uint8_t power_r) {
  unsigned long startMillis, currentMillis;
  unsigned long duration;
  distanceSensor1.startRanging();

  // Forward
  analogWrite(MOTOR1_FWD, power_l);
  analogWrite(MOTOR2_FWD, power_r);
  analogWrite(MOTOR1_BCK, 0);
  analogWrite(MOTOR2_BCK, 0);
  startMillis = millis();
  do {
    currentMillis = millis();
    if (distanceSensor1.checkForDataReady()) {
      uint16_t distance = distanceSensor1.getDistance();
      distanceSensor1.clearInterrupt();

      // Store data in buffer, using tof2 data for motor values
      tof1_time_buf[tof1_buf_len] = currentMillis;
      tof1_data_buf[tof1_buf_len] = distance;
      tof2_data_buf[tof1_buf_len] = (int16_t) power_l;
      ++tof1_buf_len;

      // Do the stunt at the target
      if (distance <= target && distance > 0) {
        duration = currentMillis - startMillis;
        break;
      }
    }
  } while (currentMillis - startMillis < timeout);

  // Backward
  analogWrite(MOTOR1_FWD, 0);
  analogWrite(MOTOR2_FWD, 0);
  analogWrite(MOTOR1_BCK, power_l);
  analogWrite(MOTOR2_BCK, power_r);  
  startMillis = millis();
  do {
    currentMillis = millis();
    if (distanceSensor1.checkForDataReady()) {
      uint16_t distance = distanceSensor1.getDistance();
      distanceSensor1.clearInterrupt();

      // Store data in buffer, using tof2 data for motor values
      tof1_time_buf[tof1_buf_len] = currentMillis;
      tof1_data_buf[tof1_buf_len] = distance;
      tof2_data_buf[tof1_buf_len] = -(int16_t) power_l;
      ++tof1_buf_len;
    }
  } while (currentMillis - startMillis < 2 * duration);

  analogWrite(MOTOR1_FWD, 0);
  analogWrite(MOTOR2_FWD, 0);
  analogWrite(MOTOR1_BCK, 0);
  analogWrite(MOTOR2_BCK, 0);
  distanceSensor1.stopRanging();
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
  float fval1, fval2, fval3, fval4;
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
    
    /**
      * RUN_PID:timeout|goal|deadband|calibration|K_p|K_i|K_d
      * Run the PID (blocking) for the duration (in milliseconds) with the given motor calibration scale and PID gains.
      */
    case RUN_PID:
      success = robot_cmd.get_next_value(duration)
        && robot_cmd.get_next_value(ival1)
        && robot_cmd.get_next_value(ival2)
        && robot_cmd.get_next_value(fval1)
        && robot_cmd.get_next_value(fval2)
        && robot_cmd.get_next_value(fval3)
        && robot_cmd.get_next_value(fval4)
      ;
      if (!success) return;
      run_pid(duration, ival1, ival2, fval1, fval2, fval3, fval4);
      break;
    
    /**
      * DATA_PID:
      * Return the data stream from the last PID run.
      * Stream format: [time{u4},distance{u2},pid{f4},motor1{u1},motor2{u1}]*
      */
    case DATA_PID:
      for (size_t i = 0; i < tof1_buf_len; ++i) {
        tx_stream.write(tof1_time_buf[i]);
        tx_stream.write(tof1_data_buf[i]);
        tx_stream.write(tof2_time_buf[i]);
        tx_stream.write(tof2_data_buf[i]);
      }
      tof1_buf_len = 0;
      tx_stream.flush();
      break;
    
    /**
      * EXTRA_DATA_PID:
      * Return the pid calculation data stream from the last PID run.
      * Stream format: [time{u4},error{f4},integrator{f4},differentiator{f4}]*
      */
    case EXTRA_DATA_PID:
      tx_stream.write(imu_buf, imu_buf_len);
      imu_buf_len = 0;
      tx_stream.flush();
      break;

    case STREAM_TOF1:
      success = robot_cmd.get_next_value(duration);
      if (!success) return;
      distanceSensor1.startRanging();
      startMillis = millis();
      do {
        currentMillis = millis();
        if (distanceSensor1.checkForDataReady()) {
          tx_stream.write(currentMillis);
          tx_stream.write(distanceSensor1.getDistance());
          distanceSensor1.clearInterrupt();
        }
      } while (currentMillis - startMillis < duration);
      distanceSensor1.stopRanging();
      tx_stream.flush();
      break;
    
    /**
      * RUN_STUNT:timeout|distance|motor1|motor2
      * Run the flip stunt for at most twice the duration (in milliseconds) with the given motor PWM values ([0..255]).
      */
    case RUN_STUNT:
      success = robot_cmd.get_next_value(duration)
        && robot_cmd.get_next_value(ival1)
        && robot_cmd.get_next_value(ival2)
        && robot_cmd.get_next_value(ival3)
      ;
      if (!success) return;
      run_stunt(duration, ival1, ival2, ival3);
      break;

    /**
      * DATA_STUNT:
      * Return the data stream from the last stunt run.
      * Stream format: [time{u4},distance{u2},motor{i2}]*
      */
    case DATA_STUNT:
      for (size_t i = 0; i < tof1_buf_len; ++i) {
        tx_stream.write(tof1_time_buf[i]);
        tx_stream.write(tof1_data_buf[i]);
        tx_stream.write(tof2_data_buf[i]);
      }
      tof1_buf_len = 0;
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
    shutdown();
    Serial.println("Disconnected");
  }
}
