#include <iostream>
#include <math.h>
#include <random>
#include <vector>

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

float probNormalDistribution(float mean, float variance, float x) {
  return exp(-0.5 * ((x - mean) * (x - mean)) / (variance)) /
         (sqrt(2 * M_PI * variance));
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
  return;
}

// 设想住在一个白天天气为晴、多云或者雨的地方。天气转移函数是如下的转移表所示的马尔可夫链
void q2() {

  // Sunny = 0 ; cloudy = 1; rainy = 2;
  vector<vector<float>> weather_tf{
      {0.8, 0.2, 0.0}, {0.4, 0.4, 0.2}, {0.2, 0.6, 0.2}};

  // 1. 第一天是晴， 求接下来第2天多云, 第3天是多云， 第4天是雨的概率
  float p_rain = weather_tf[0][1] * weather_tf[1][1] * weather_tf[1][2];
  // cout << "第3天多云, 第4天雨的概率是 : " << p_rain << endl;

  // 2. 天气序列模拟器
  srand((unsigned)time(NULL));
  // 通用公式：a+rand () % n;   其中：a 为范围起始位置，n 为整数的范围
  float today_rand;
  int today_weather, yest_weather = 0;
  vector<int> weather_sim;
  vector<float> today_tf;
  int i, n = 100;
  for (i = 0; i < n; i++) {
    today_rand = rand() / float(RAND_MAX);
    today_tf = weather_tf[yest_weather];
    if (today_rand <= today_tf[0]) {
      today_weather = 0;
    } else if (today_rand <= (today_tf[0] + today_tf[1])) {
      today_weather = 1;
    } else {
      today_weather = 2;
    }
    weather_sim.push_back(today_weather);
    yest_weather = today_weather;
    // cout << "今天的随机数是 : "  << today_rand << "\t天气是 :" <<
    // today_weather << endl;
  }

  // 3. 平稳分布
  // 闭式方程是 状态转移矩阵特征值是1时对应的特征向量
  vector<float> stable_tf{1.0, 0.0, 0.0};
  float p_sunny, p_rainy, p_cloudy;
  for (i = 0; i < n; i++) {
    p_sunny = stable_tf[0] * weather_tf[0][0] +
              stable_tf[1] * weather_tf[1][0] + stable_tf[2] * weather_tf[2][0];
    p_cloudy = stable_tf[0] * weather_tf[0][1] +
               stable_tf[1] * weather_tf[1][1] +
               stable_tf[2] * weather_tf[2][1];
    p_rainy = stable_tf[0] * weather_tf[0][2] +
              stable_tf[1] * weather_tf[1][2] + stable_tf[2] * weather_tf[2][2];

    stable_tf[0] = p_sunny;
    stable_tf[1] = p_cloudy;
    stable_tf[2] = p_rainy;
  }
  // for (auto num : stable_tf)
  //   cout << "天气的平稳分布是 : " << num << endl;

  // 4. 计算给定今天的天气时昨天的概率表
  today_weather = 1;
  vector<float> p_yest{0.2, 0.4, 0.4};
  float p_full = p_yest[0] * weather_tf[0][today_weather] +
                 p_yest[1] * weather_tf[1][today_weather] +
                 p_yest[2] * weather_tf[2][today_weather];
  for (i = 0; i < 3; i++) {
    p_yest[i] = (p_yest[i] * weather_tf[i][today_weather]) / p_full;
  }

  // for (auto num : p_yest)
  //   cout << "昨天的天气概率" << num << endl;

  // 5.使用带误差的观测器
  vector<vector<float>> sensor_matrix = {
      {0.6, 0.4, 0.0}, {0.3, 0.7, 0.0}, {0.0, 0.0, 1.0}};
  int sensor_weather = 0;
  vector<float> p_today = weather_tf[2];
  vector<float> p_torrom = {0, 0, 0.0, 0.0};
  vector<float> sensor_torrom = {0.0, 0.0, 0.0};
  p_full = p_today[0] * sensor_matrix[0][sensor_weather] +
           p_today[1] * sensor_matrix[1][sensor_weather] +
           p_today[2] * sensor_matrix[2][sensor_weather];

  for (i = 0; i < 3; i++) {
    p_today[i] = (p_today[i] * sensor_matrix[i][sensor_weather]) / p_full;
  }

  for (i = 0; i < 3; i++) {
    p_torrom[i] = p_today[0] * weather_tf[0][i] +
                  p_today[1] * weather_tf[1][i] + p_today[2] * weather_tf[2][i];
  }

  for (i = 0; i < 3; i++) {
    sensor_torrom[i] = p_torrom[0] * sensor_matrix[0][i] +
                       p_torrom[1] * sensor_matrix[1][i] +
                       p_torrom[2] * sensor_matrix[2][i];
  }
  // cout << "第5天传感器为晴的概率 : " << sensor_torrom[0] << endl;

  // 6.
  p_yest = {1.0, 0.0, 0.0};
  vector<float> sensor_today = {0, 0, 0};
  int sensor_arr[3] = {0, 0, 2};
  int j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      p_today[j] = p_yest[0] * weather_tf[0][j] + p_yest[1] * weather_tf[1][j] +
                   p_yest[2] * weather_tf[2][j];
    }
    sensor_weather = sensor_arr[i];
    p_full = p_today[0] * sensor_matrix[0][sensor_weather] +
             p_today[1] * sensor_matrix[1][sensor_weather] +
             p_today[2] * sensor_matrix[2][sensor_weather];

    for (j = 0; j < 3; j++) {
      p_yest[j] = (p_today[j] * sensor_matrix[j][sensor_weather]) / p_full;
      cout << "基于当天，第 " << i + 1 << " 天的天气概率" << p_yest[j] << endl;
    }
  }

  return;
}

// 将贝叶斯准则应用到高斯情况
void q3() {

  // 1. PDF for gaussian distrubution
  float x_init = 1000.0;
  float x_variance = 900;
  float z_init = 1100.0;
  float z_variance = 100;

  float prior_x = probNormalDistribution(x_init, x_variance, z_init);
  float prior_z = probNormalDistribution(z_init, z_variance, z_init);
}

int main(int argc, char **argv) {

  q3();
  return 0;
}
