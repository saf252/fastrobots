#ifndef _PID_H_
#define _PID_H_

struct PID {
  float target;
  float k_p;
  float k_i;
  float k_d;
};

extern float imu_buf[];
extern size_t imu_buf_len;

float pid(unsigned long currentMillis, float currentDistance, PID& params, bool reset) {
  static unsigned long previousMillis;
  static unsigned short previousDistance;
  static float integrator;
  float differentiator;
  float error;

  error = currentDistance - params.target;
  if (reset) {
    integrator = 0;
    differentiator = 0;
  } else {
    float dt = currentMillis - previousMillis;
    integrator = integrator + error * dt;
    differentiator = (currentDistance - previousDistance) / dt;
  }
  previousMillis = currentMillis;
  previousDistance = currentDistance;

  imu_buf[imu_buf_len++] = *(float*) &currentMillis;
  imu_buf[imu_buf_len++] = error;
  imu_buf[imu_buf_len++] = integrator;
  imu_buf[imu_buf_len++] = differentiator;

  return params.k_p * error + params.k_i * integrator + params.k_d * differentiator;
}

#endif // _PID_H_