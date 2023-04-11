// #include "q1.h"
#include <iostream>
#include <math.h>

using namespace std;

void motionModelVelocity(float control[2], float current[3], float desired[3]) {

  float d_t = 0.1;

  float d_x = current[0] - desired[0];
  float d_y = current[1] - desired[1];
  float a = 0.5 * (d_x * cos(current[2]) + d_y * sin(current[2])) /
            (d_y * cos(current[2]) - d_x * sin(current[2]));

  float x_hat = 0.5 * (current[0] + desired[0]) + a * (current[1] - desired[1]);
  float y_hat = 0.5 * (current[1] + desired[1]) + a * (current[0] - desired[0]);
  float r_hat = sqrt(((current[0] - x_hat) * (current[0] - x_hat)) +
                     ((current[1] - y_hat) * (current[1] - y_hat)));

  float det_theta = atan2(desired[1] - y_hat, desired[0] - x_hat) -
                    atan2(current[1] - y_hat, current[0] - x_hat);

  float v_predic = det_theta * r_hat / d_t;
  float w_predic = det_theta / d_t;
  float r_predic = (desired[2] - current[2]) / d_t - w_predic;

  return;
}

float probNormalDistribution(float a, float b) {
  return exp(-0.5 * (a * a) / (b * b)) / (sqrt(2 * M_PI * b * b));
}

float probTriangularDistribution(float a, float b) {
  return fmax(0, 1 / (sqrt(6) * b) - fabs(a) / (6 * b * b));
}

// 传感器测量0-3m， 故障时只输出小于1m， 故障的先验为0.01.
void q1() {

  // 1.1： 测试N次， 都小于1m， 对于N=1到10， 故障的后验概率是多少
  float prior_BadSensor = 0.01;
  float prior_GoodSensor = (1.0 - prior_BadSensor);
  float z_BadSensor = 1.0;
  float z_GoodSensor = 1.0 / 3.0;

  float posterior_BadSensor;
  for (int i = 0; i < 10; i++) {
    posterior_BadSensor =
        (prior_BadSensor * z_BadSensor) /
        (prior_GoodSensor * z_GoodSensor + prior_BadSensor * z_BadSensor);
    prior_BadSensor = posterior_BadSensor;
    prior_GoodSensor = 1.0 - prior_BadSensor;
    cout << "[INFO] The Posterior probabilics of Bad Sensor is  "
         << posterior_BadSensor << endl;
  }

  // 1.2:
  // 设想住在一个白天天气为晴、多云或者雨的地方。天气转移函数是如下的转移表所示的马尔可夫链
}

int main() {

  q1();
  return 0;
}