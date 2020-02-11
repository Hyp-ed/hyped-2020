/*
 * Author: Justus Rudolph, George Karabassis
 * Organisation: HYPED
 * Date: 30/03/2019
 * Description: Header for Kalman filter (interface for filter and filter setup)
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

#ifndef NAVIGATION_KALMAN_FILTER_HPP_
#define NAVIGATION_KALMAN_FILTER_HPP_

#include <random>
#include "../lib/Eigen/Dense"

#include "data/data.hpp"
#include "utils/system.hpp"
#include "utils/math/kalman_multivariate.hpp"

using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace hyped {

using data::NavigationType;
using data::NavigationVector;
using utils::System;
using utils::math::KalmanMultivariate;

namespace navigation {

class KalmanFilter {
    // const float kInitialErrorVar;
    // const float kStateTransitionVar;
    // const float kTubeMeasurementVar;
    // const float kElevatorMeasurementVar;
    // const float kStationaryMeasurementVar;

  public:
    /*
    * Constructor call for a kalman filter
    * n : dimension of the state
    * m : dimension of the measurement
    * k : dimension of control   NOTE: will not be used in this implementation
    */
  
    KalmanFilter(int n, int m, int k = 0);

    /*
    * Constructor call for a kalman filter
    * x : state
    * A : Transition Matrix
    * P : Covariance
    * Q : Random uncertainty covariacne
    * H : Measurement Matrix
    * R : Sensor Noise Covariance
    */
    void init(VectorXf x, MatrixXf P, MatrixXf Q, MatrixXf R);

    // set initial state and covariance
    void setInitial(VectorXf init);  // ! Add initial covariance matrix - fixed

    // get state
    VectorXf getState();

    // get covariacne
    MatrixXf getCovariance();

    // filter using 5 KF equations
    /*
    * dt : Time interval
    * s  : state
    * z  : sensor measurement vector
    */
    void filter(float dt, VectorXf s, VectorXf z);  // !! Two variables missing, update this! (real life values) - fixed

  public:
    // f1 : predict state
    void predictState();
    // f2 : estimate state (update based on measurement)
    void estimateState(MatrixXf K, VectorXf z);
    // f3 : kalman gain
    MatrixXf kalmanGain();
    // f4 : predict state covariance
    void predictCovariance();
    // f5 : estimate state covariance
    void estimateCovariance(MatrixXf K);
    // f10 : update state transition;
    void updateStateTransition(float Dt);  // ! lowercase d
    // f11 : update state transition;
    void updateSensorNoise(MatrixXf R);  // Find out - how to update R?
    // f12 : get data from sensors (may have parameters according to sensors spec)
    //       Will set the measurement vector
    // Vector get_data();
    // f13 : get measurement
    void setMeasurement(VectorXf z);
    // f14 : get measurement
    VectorXf getMeasurement();
    // Setting Matrix H;
    MatrixXf setMeasurementMatrix();

    int n_;
    int m_;
    int k_;

    VectorXf x_;  // current state (n x 1)
    VectorXf z_;  // measurement (m x 1)
    MatrixXf A_;  // state transition matrix (n x n)
    MatrixXf P_;  // state covariance matrix (n x n)
    MatrixXf Q_;  // state covariance noise matrix (n x n)
    MatrixXf H_;  // dimension change matrix (m x n)
    MatrixXf R_;  // measurement noise matrix (m x m)
    MatrixXf I_;  // identity matrix (n x n)
};

}}  // namespace hyped::navigation
  
#endif  // NAVIGATION_KALMAN_FILTER_HPP_
