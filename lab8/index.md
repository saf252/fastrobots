---
---

# Lab 8: Stunts!

_I'm honestly kind of struggling right now, and I can't find my bluetooth dongle, so I don't have videos yet as I haven't been able to run this on the robot.
Hopefully I'll be able to get that sorted soon._

## Setup

I separated the task running and data transmitting into separate Robot Commands, as in previous labs.
I disengaged my PID control and did not use a Kalman Filter, so the code on the Artemis was pretty straightforward:

```cpp
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
  } while (currentMillis - startMillis < duration);

  analogWrite(MOTOR1_FWD, 0);
  analogWrite(MOTOR2_FWD, 0);
  analogWrite(MOTOR1_BCK, 0);
  analogWrite(MOTOR2_BCK, 0);
  distanceSensor1.stopRanging();
}
```

and I could run the stunt and collect the ToF and PWM data from Jupyter with

```py
##
time.sleep(5)
timeout_s = 10
target_m = 0.5
power_l = 255
power_r = power_l
ble.send_command(CMD.RUN_STUNT, f"{int(timeout_s*1000)}|{int(target_m*1000)}|{power_l}|{power_r}")
time.sleep(timeout_s)
##
fut = ble_rx_stream(ble, 'RX_STREAM')
ble.send_command(CMD.DATA_STUNT, "")
stream_data = list(unpack_stream('LHh*', await fut))
```

## Stunts

_I will put my graphs and videos here as soon as I am actually able to create them._
