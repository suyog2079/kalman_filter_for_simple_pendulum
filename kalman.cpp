#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <fstream>
#include <iostream>
#include <ostream>
#include <random>
#include <sys/types.h>

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
    omega = omega - period * 9.81 * sin(theta);
  }
};

/**
 * @class RandomVector
 * @brief I took this directly from ujwol. Don't know how it works.
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
  Eigen::Matrix<double, 2, 2> A; // state transition matrix (model)
  Eigen::Matrix<double, 2, 2> C; // measurement matrix

  Eigen::Matrix<double, 2, 1> state;
  Eigen::Matrix<double, 2, 2> state_cov;

  Eigen::Matrix<double, 2, 1> measurement;
  Eigen::Matrix<double, 2, 2> measurement_cov;

  Eigen::Matrix<double, 2, 2> v; // this is the sensor noise
  Eigen::Matrix<double, 2, 2> w; // this is measurement noise

  double period = 0.1;
  KalmanFilter() {
    state << 0.5, 0.2;

    state_cov.setIdentity();
    measurement_cov.setIdentity();

    w.setZero();
    v << 0.09, 0.01, 0.01, 0.3;
  }

  void update(Sensor s) {
    C.setIdentity();

    Eigen::Matrix<double, 2, 1> sensor;
    sensor << s.theta, s.omega;

    Eigen::Matrix<double, 2, 2> S =
        C * state_cov * C.transpose() + v;

    Eigen::Matrix<double, 2, 2> kalman_gain =
        state_cov * C.transpose() * S.inverse();

    state = state + kalman_gain * (sensor - C * state);

    Eigen::Matrix<double, 2, 2> I;
    I.setIdentity();

    state_cov = (I - kalman_gain * C) * state_cov;
  }

  void predict() {
    A << 1, period, (-period * 9.81 * cos(state(0, 0))), 1;

    state(0, 0) = state(0, 0) + period * state(1, 0);
    state(1, 0) = state(1, 0) - period * 9.81 * sin(state(0, 0));

    state_cov = A * state_cov * A.transpose() + w;
  }
};

std::ostream &operator<<(std::ostream &os, const Pendulum p) {
  return os << p.time * p.period << "\t" << p.theta;
}

std::ostream &operator<<(std::ostream &os, const Sensor s) {
  return os << "\t" << s.theta;
}

int main() {
  Pendulum P(0.5,0.2);
  Sensor S;
  KalmanFilter K;

  std::ofstream fs("data.txt", std::ios::out);

  for (int i = 0; i < 100; i++) {
    P.tick();
    K.predict();
    S.update_sensor_data(P);
    fs << S;
    K.update(S);
    fs << "\t" << K.state(0, 0);
    fs << "\n";
  }
  return 0;
}
