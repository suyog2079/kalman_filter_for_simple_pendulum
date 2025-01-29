#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <random>

class Pendulum {
public:
  double theta;
  double omega;

  int time = 0;
  double period = 0.1;

  Pendulum(double initial_theta, double initial_omega)
      : theta(initial_theta), omega(initial_omega) {};

  void tick() {
    time++;

    theta = theta + period * omega;
    omega = omega - period * 9.8 * sin(theta);
  }
};

/**
 * @class RandomVector
 * @brief I took this directly from ujwal. Don't know how it works.
 *
 */
class RandomVector {
public:
  float m1_, m2_, v1_, c12_, v2_;

  RandomVector(float m1, float m2, float v1, float c12, float v2) {
    m1_ = m1;
    m2_ = m2;
    v1_ = v1;
    v2_ = v2;
    c12_ = c12;
  }

  float l11, l22, l12;

  float x, y;

  void set_vector() {
    // Cholesky Decompositinn
    l11 = sqrt(v1_);
    l12 = c12_ / l11;

    if (v2_ - l12 * l12 < 0)
      throw -1;

    l22 = sqrt(v2_ - l12 * l12);

    float z1, z2;

    // Define random generator with Gaussian distribution
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> dist(0.0, 1.0);

    z1 = dist(generator);
    z2 = dist(generator);

    x = l11 * z1;
    y = l12 * z1 + l22 * z2;

    x += m1_;
    y += m2_;
  }
};

class Sensor {
public:
  double theta;
  double omega;

public:
  Sensor() : theta(0), omega(0) {}

  void update_sensor_data(const Pendulum &p) {
    RandomVector r(0, 0, 0.09, 0.01, 0.3);
    r.set_vector();

    theta = p.theta + r.x;
    omega = p.omega + r.y;
  }
};

class KalmanFilter {
public:
  Eigen::Matrix<double, 2, 2> model_jacobian;
  Eigen::Matrix<double, 2, 2> sensor_jacobian;

  Eigen::Matrix<double, 2, 1> state;
  Eigen::Matrix<double, 2, 2> state_cov;

  Eigen::Matrix<double, 2, 1> measurement;
  Eigen::Matrix<double, 2, 2> measurement_cov;

  Eigen::Matrix<double, 2, 2> v; // this is the sensor covariance
  Eigen::Matrix<double, 2, 2> w;

  double period = 0.1f;
  KalmanFilter() {
    state(0, 0) = 0.5;
    state(0, 1) = 0.2;

    state_cov = Eigen::Matrix<double, 2, 2>::Identity();
    measurement_cov = Eigen::Matrix<double, 2, 2>::Identity();

    w << 0, 0, 0, 0;
    v << 0.09, 0.01, 0.01, 0.3;
  }

  void update(Sensor s) {
    sensor_jacobian << 1, 0, 0, 1;

    Eigen::Matrix<double, 2, 1> sensor;
    sensor << s.theta, s.omega;

    Eigen::Matrix<double, 2, 2> A =
        sensor_jacobian * state_cov * sensor_jacobian.transpose() + v;

    Eigen::Matrix<double, 2, 1> kalman_gain =
        state_cov * sensor_jacobian.transpose() * A.inverse();

    state = state + kalman_gain * (sensor - sensor_jacobian * state);

    Eigen::Matrix<double, 2, 2> I = Eigen::Matrix<double, 2, 2>::Identity();

		state_cov = (I - kalman_gain * sensor_jacobian) * state_cov;
  }

  void predict() {

    model_jacobian << 1, period, (-period * 9.81 * cos(state(0, 0))), 1;

    state(0, 0) = state(0, 0) + period * state(0, 1);
    state(1, 0) = state(1, 0) - period * 9.8 * sin(state(0, 0));

    state_cov = model_jacobian * state_cov * model_jacobian.transpose() + w;
  }
};

int main() {}
