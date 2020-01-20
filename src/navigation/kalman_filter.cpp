/*
 * Author: George Karabassis
 * Organisation: HYPED
 * Date: 30/03/2019
 * Description: Kalman filter (interface for filter and filter setup)
 *
 *  Copyright 2019 HYPED
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *  except in compliance with the License. You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software distributed under
 *  the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *  either express or implied. See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "navigation/kalman_filter.hpp"

namespace hyped {
namespace navigation {

// constexpr float KalmanFilter::kInitialErrorVar;
// constexpr float KalmanFilter::kStateTransitionVar;
// constexpr float KalmanFilter::kTubeMeasurementVar;
// constexpr float KalmanFilter::kElevatorMeasurementVar;
// constexpr float KalmanFilter::kStationaryMeasurementVar;

KalmanFilter::KalmanFilter(int n, int m, int k)
    : n_(n)
    , m_(m)
    , k_(k)
{
    z_.resize(m);
    x_.resize(n);
    A_.resize(n, n);
    P_.resize(n, n);
    Q_.resize(n, n);
    R_.resize(m, m);
    I_.resize(n, n);
    H_.resize(m, n);
}

void KalmanFilter::init(VectorXf x, MatrixXf P, MatrixXf Q, MatrixXf R)
{
    x_ = x;
    P_ = P;
    Q_ = Q;
    H_ = setMeasurementMatrix();
    R_ = R;
    I_ = MatrixXf::Identity(n_, n_);

    // creating the init MatrixXf. Find a way to add this to the MatrixXf_lib as a 
    // method and not allow the user to assign a new value to it.

    // creating the init MatrixXf. Find a way to add this to the MatrixXf_lib as a
    // method and not allow the user to assign a new value to it.
}

void KalmanFilter::setInitial(VectorXf init)
{
    x_ =  init;
}

MatrixXf KalmanFilter::setMeasurementMatrix()
{
    MatrixXf H(m_, n_);
    H = MatrixXf::Zero(m_, n_);

    for (int i = 0; i < m_; i++) {
        H(i, n_ - m_ + i) = 1;
    }

    return H;
}

VectorXf KalmanFilter::getState()
{
    return x_;
}

MatrixXf KalmanFilter::getCovariance()
{
    return P_;
}

VectorXf KalmanFilter::getMeasurement()
{
    return z_;
}

void KalmanFilter::updateSensorNoise(MatrixXf R)
{
    // some sort of calclulation take place here.

    R_ = R;
}

void KalmanFilter::filter(float dt, VectorXf s, VectorXf z)
{
    KalmanFilter::updateStateTransition(dt);

    KalmanFilter::predictState();
    KalmanFilter::predictCovariance();

    MatrixXf K(n_, n_);
    K = KalmanFilter::kalmanGain();

    // KalmanFilter::get_data(); // manipulate sensor data and set measurement VectorXf

    KalmanFilter::estimateState(K, z);
    KalmanFilter::estimateCovariance(K);
}

void KalmanFilter::predictState()
{
    x_ = A_ * x_;
}

void KalmanFilter::predictCovariance()
{
    P_ = A_ * P_ * A_.transpose() + Q_;
}

MatrixXf KalmanFilter::kalmanGain()
{
    MatrixXf K(n_, m_);
    K = (P_ * H_.transpose()) * (H_ * P_ * H_.transpose() + R_).inverse();
    return K;
}

void KalmanFilter::estimateState(MatrixXf K, VectorXf z)
{
    x_ = x_ + K * (z - H_ * x_);
}

void KalmanFilter::estimateCovariance(MatrixXf K)
{
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::updateStateTransition(float dt)
{
    MatrixXf A(n_, n_);
    A = MatrixXf::Zero(n_, n_);

    VectorXf s(n_);
    s << 1.0, dt, dt*dt/2;  // {displacement, velocity, acceleration placeholders}

    for (int i = 0; i < n_; i++) {
        for (int j = i; j < n_; j++) {
            if ((j - i) % m_ == 0) {
                A(i, j) = s((j - i)/m_);
            }
        }
    }
    A_ = A;
}

void KalmanFilter::setMeasurement(VectorXf z)
{
    z_ = z;
}

}}  // namespace hyped navigation
