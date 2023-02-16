#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "BLETxStream.h"
#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SENSOR1_ADDR 0x32
#define SENSOR2_SHUT D8

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

//////////// Global Variables ////////////

enum CommandTypes
{
    GET_TOF_TIME,
};

void get_tof_time(unsigned long duration)
{
    uint16_t distance;
    distanceSensor1.startRanging();
    distanceSensor2.startRanging();
    unsigned long startMillis = millis();
    unsigned long currentMillis = millis();
    while (currentMillis - startMillis < duration)
    {
        if (distanceSensor1.checkForDataReady())
        {
            distance = distanceSensor1.getDistance();
            distanceSensor1.clearInterrupt();
            tx_stream.write(currentMillis);
            tx_stream.write(distance);
            Serial.printf("%li -> %hu\n", currentMillis, distance);
        }
        if (distanceSensor2.checkForDataReady())
        {
            distance = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            tx_stream.write(~currentMillis);
            tx_stream.write(distance);
            Serial.printf("%li -> %hu\n", ~currentMillis, distance);
        }
        currentMillis = millis();                
    }
    tx_stream.flush();
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

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Send timestamped ToF data for the number of milliseconds in the command string
         * If the timestamp t is positive then the data is from Sensor 1, otherwise the data is from Sensor 2 at time ~t
         */
        case GET_TOF_TIME:
            int duration;
            success = robot_cmd.get_next_value(duration);
            if (!success)
                return;
            get_tof_time(duration);
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
        }

        Serial.println("Disconnected");
    }
}
