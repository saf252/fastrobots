#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "BLETxStream.h"
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "ICM_20948.h"
#include <math.h>

#define SENSOR1_ADDR 0x32
#define SENSOR2_SHUT D8

#define AD0_VAL 1

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "487dcfb9-3553-41d8-9c89-a2d29a7ec943"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_STREAM "51cc72fd-5d0a-48f9-9d9f-7829f37e1f02"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLECharacteristic tx_characteristic_stream(BLE_UUID_TX_STREAM, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
BLETxStream tx_stream(&tx_characteristic_stream);

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;

ICM_20948_I2C myICM;

bool recording = false;

float imu_buf[7 * 960];
size_t imu_buf_len = 0;

unsigned long tof1_time_buf[160];
uint16_t tof1_data_buf[160];
size_t tof1_buf_len = 0;

unsigned long tof2_time_buf[160];
uint16_t tof2_data_buf[160];
size_t tof2_buf_len = 0;

//////////// Global Variables ////////////

enum CommandTypes
{
    GET_IMU_TIME,
    BUF_ALL_TIME,
    GET_IMU_BUF,
    GET_TOF_BUF,
    START_RECORDING_STREAM,
    STOP_RECORDING_STREAM,
};

void get_imu_time(unsigned long duration)
{
    unsigned long startMillis = millis();
    unsigned long currentMillis = millis();
    while (currentMillis - startMillis < duration)
    {
        if (myICM.dataReady())
        {
            myICM.getAGMT();
            imu_buf[imu_buf_len++] = *(float *) &currentMillis;
            imu_buf[imu_buf_len++] = myICM.accX();
            imu_buf[imu_buf_len++] = myICM.accY();
            imu_buf[imu_buf_len++] = myICM.accZ();
            imu_buf[imu_buf_len++] = myICM.gyrX();
            imu_buf[imu_buf_len++] = myICM.gyrY();
            imu_buf[imu_buf_len++] = myICM.gyrZ();
        }
        currentMillis = millis();                
    }
    tx_stream.write(imu_buf, imu_buf_len);
    imu_buf_len = 0;
    tx_stream.flush();
}

void buf_all_time(unsigned long duration)
{
    uint16_t distance;
    distanceSensor1.startRanging();
    distanceSensor2.startRanging();
    unsigned long startMillis = millis();
    unsigned long currentMillis = millis();
    while (currentMillis - startMillis < duration)
    {
        if (myICM.dataReady())
        {
            myICM.getAGMT();
            imu_buf[imu_buf_len++] = *(float *) &currentMillis;
            imu_buf[imu_buf_len++] = myICM.accX();
            imu_buf[imu_buf_len++] = myICM.accY();
            imu_buf[imu_buf_len++] = myICM.accZ();
            imu_buf[imu_buf_len++] = myICM.gyrX();
            imu_buf[imu_buf_len++] = myICM.gyrY();
            imu_buf[imu_buf_len++] = myICM.gyrZ();
        }
        if (distanceSensor1.checkForDataReady())
        {
            distance = distanceSensor1.getDistance();
            distanceSensor1.clearInterrupt();
            tof1_time_buf[tof1_buf_len] = currentMillis;
            tof1_data_buf[tof1_buf_len] = distance;
            ++tof1_buf_len;
        }
        if (distanceSensor2.checkForDataReady())
        {
            distance = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            tof2_time_buf[tof2_buf_len] = currentMillis;
            tof2_data_buf[tof2_buf_len] = distance;
            ++tof2_buf_len;
        }
        currentMillis = millis();                
    }
    distanceSensor1.stopRanging();
    distanceSensor2.stopRanging();
}

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    int duration;
    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Send timestamped IMU data for the number of milliseconds in the command string
         * The data is sent as: [timestamp, accX, accY, accZ, gyrX, gyrY, gyrZ]
         */
        case GET_IMU_TIME:
            success = robot_cmd.get_next_value(duration);
            if (!success)
                return;
            get_imu_time(duration);
            break;
        
        /* 
         * Buffer timestamped IMU and ToF data for the number of milliseconds in the command string
         */
        case BUF_ALL_TIME:
            success = robot_cmd.get_next_value(duration);
            if (!success)
                return;
            buf_all_time(duration);
            break;

        /*
         * Send timestamped IMU data from the buffer
         * The data is sent as: [timestamp, accX, accY, accZ, gyrX, gyrY, gyrZ]
         */
        case GET_IMU_BUF:
            tx_stream.write(imu_buf, imu_buf_len);
            imu_buf_len = 0;
            tx_stream.flush();
            break;
        
        /*
         * Send timestamped ToF data from the buffer for the sensor specified in the command (1 or 2)
         * The data is sent as: [timestamp, distance]
         */
        case GET_TOF_BUF:
            int sensor;
            success = robot_cmd.get_next_value(sensor);
            if (!success)
                return;
            if (sensor == 1)
            {
                for (size_t i = 0; i < tof1_buf_len; ++i)
                {
                    tx_stream.write(tof1_time_buf[i]);
                    tx_stream.write(tof1_data_buf[i]);
                }
                tof1_buf_len = 0;
            }
            else if (sensor == 2)
            {
                for (size_t i = 0; i < tof2_buf_len; ++i)
                {
                    tx_stream.write(tof2_time_buf[i]);
                    tx_stream.write(tof2_data_buf[i]);
                }
                tof2_buf_len = 0;
            }
            tx_stream.flush();
            break;
        
        /*
         *  Begin recording sensor data
         */
        case START_RECORDING_STREAM:
            recording = true;
            distanceSensor1.startRanging();
            distanceSensor2.startRanging();
            break;
        
        /*
         *  Finish recording sensor data and send it over Bluetooth
         */
        case STOP_RECORDING_STREAM:
            recording = false;
            tx_stream.flush();
            distanceSensor1.stopRanging();
            distanceSensor2.stopRanging();
            break;
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Wire.begin();

    Serial.begin(115200);

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_stream);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    uint8_t init_data[] = { 1 };
    tx_characteristic_stream.writeValue(init_data, 1);

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();

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

    myICM.begin(Wire, AD0_VAL);

    // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

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

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {

            // Read data
            read_data();

            // Collect IMU data
            if (recording) {
                while (!myICM.dataReady());
                tx_stream.write(millis());
                myICM.getAGMT();
                tx_stream.write(myICM.accX());
                tx_stream.write(myICM.accY());
                tx_stream.write(myICM.accZ());
                tx_stream.write(myICM.gyrX());
                tx_stream.write(myICM.gyrY());
                tx_stream.write(myICM.gyrZ());
                while (!distanceSensor1.checkForDataReady());
                tx_stream.write(millis());
                tx_stream.write(distanceSensor1.getDistance());
                distanceSensor1.clearInterrupt();
                while (!distanceSensor2.checkForDataReady());
                tx_stream.write(millis());
                tx_stream.write(distanceSensor2.getDistance());
                distanceSensor2.clearInterrupt();
            }
        }

        Serial.println("Disconnected");
    }
}
