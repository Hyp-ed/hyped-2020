/*
 * Author: George Karabassis
 * Co-Author: 
 * Organisation: HYPED
 * Date: 05/04/2019
 * Description: Navigation algorithm.
 *
 *    Copyright 2019 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include "navigation/kalman_filter.hpp"
#include "navigation/navigation.hpp"
#include "IMUs_faker.hpp"

namespace hyped {
namespace navigation {

  Navigation::Navigation(Logger& log, int n, int m, int k, KalmanFilter& KF_)
    : log_(log)
    , sys_(System::getSystem())
    , data_(Data::getInstance())
    , KF_(KF_)
    , status_(ModuleStatus::kStart)
    , dt_(0)
    , m_(m)
    , n_(n)
    , k_(k)
  {}

  void Navigation::IMUQuerying(int i)
  {
    // TODO(george): Make it better.

    // store IMU data
    ImuDataPointArray sensorReadings = data_.getSensorsImuData();
    // Wheel encoders data is empty for now. Change the function as soon as there is one available in sensors
    ImuDataPointArray wheelEncoders_data = data_.getSensorsImuData();

    // update timestamp for new KF update
    Navigation::timestampUpdate();

    int failedIMUs = Navigation::OutlierDetection(sensorReadings, wheelEncoders_data);

    if (failedIMUs > 1) {  // more than one IMUs failed
      status_ = ModuleStatus::kCriticalFailure;
      log_.ERR("NAV", "More than 1 IMU had failed, entering kCriticalFailure");
    } else {  // Kalman filter update
      Navigation::KFUpdate(i);
    }
  }

  int Navigation::OutlierDetection(ImuDataPointArray& sensorReadings, ImuDataPointArray& wheelEncoders_data)
  {
    // TODO(Outlier_lads): Check if there can be any improvements.
    // For now, state_acc.txt is used as fake data for testing/demo purposes.

    float IMUdataMedian = 0.0;  // IMU data median
    float IMUdataMean = 0.0;  // IMU data mean
    float meanAD = 0.0;  // mean absolute derivation
    float medAD = 0.0;  // median absolute derivation
    int nFailedIMUs = 0;  // Total Failed IMUs
    vector<float> modZscore;  // Modified Z-score (source: IBM)
    vector<float> IMU_measurements;

    for (int i = 0; i < data::Sensors::kNumImus; ++i) {
      // if faulty imu value, replace it with 0 acceleration  // find out how to check whether IMU is faulty.
      // increment failedIMUs
      IMU_measurements.at(data::Sensors::kNumImus + i) = sensorReadings.value[i].acc[0];  // currently set to 0 (x axis). Will update it to contain all 3 axes
      IMUdataMean = IMUdataMean + sensorReadings.value[i].acc[0];  // currently set to 0 (x axis). Will update it to contain all 3 axes
    }
    sort(IMU_measurements.begin(), IMU_measurements.end());  // sorting IMU data

    // IMU data mean
    IMUdataMean = IMUdataMean/IMU_measurements.size();

    // IMU data median
    if (data::Sensors::kNumImus % 2 == 0) {
      IMUdataMedian = 3*IMU_measurements.at(IMU_measurements.size()/2 - 1)/2 - IMU_measurements.at(IMU_measurements.size()/2)/2;
    } else {
      IMUdataMedian = IMU_measurements.at((IMU_measurements.size() - 1)/2);
    }

    // IMU absolute data mean derivation
    float nAbsDataMeanDiff = 0.0;
    for (int i=0; i < IMU_measurements.size(); i++) {
      nAbsDataMeanDiff = nAbsDataMeanDiff + abs(IMU_measurements.at(i) - IMUdataMean);
    }
    meanAD = nAbsDataMeanDiff/IMU_measurements.size();

    // IMU absolute data median derivation
    vector<float> nAbsDataMedianDiff;
    for (int i=0; i < IMU_measurements.size(); i++) {
      nAbsDataMedianDiff.at(i) = abs(IMU_measurements.at(i) - IMUdataMedian);
    }
    sort(nAbsDataMedianDiff.begin(), nAbsDataMedianDiff.end());

    if (data::Sensors::kNumImus % 2 == 0) {
      medAD = 3*nAbsDataMedianDiff.at(nAbsDataMedianDiff.size()/2 - 1)/2 - nAbsDataMedianDiff.at(nAbsDataMedianDiff.size()/2)/2;
    } else {
      medAD = nAbsDataMedianDiff.at((nAbsDataMedianDiff.size() - 1)/2);
    }

    // Calculating Modified Z scores
    if (medAD != 0) {
      for (int i=0; i < IMU_measurements.size(); i++) {
        modZscore.at(i) = (IMU_measurements.at(i) - IMUdataMedian)/(1.486 * medAD);
      }
    }
    else {
      for (int i=0; i < IMU_measurements.size(); i++) {
        modZscore.at(i) = (IMU_measurements.at(i) - IMUdataMedian)/(1.253314 * meanAD);
      }
    }

    // Replace outliers exceeding 3.5 sd
    for (int i=0; i < modZscore.size(); i++) {
      if (modZscore.at(i) > 3.5){
        IMU_measurements.at(i) = IMUdataMedian;
      }
    }

    // Storing final IMU_measurements into VectorXf
    for (int i=0; i < IMU_measurements.size(); i++) {
      z_(i) = IMU_measurements.at(i);
    }

    // For now, there are 0 failed IMUS for demo purposes
    return nFailedIMUs;
  };

  void Navigation::KFUpdate(int i)
  {
    // The way KF works is subject to change to use the stacking KF technique.
    // Will work through it soon
    
    VectorXf s(n_);
    s = KF_.getState();

    // for demo purposes only
    demoIMUData IMUdemo = demoIMUData("fake_state_acc.txt", m_);
    vector< vector<float> > data = IMUdemo.getData();

    for (int d = 0; d < m_; d++) {
        z_(d) = data[i][d+1];
    }
    
    // Filtering - predict next state.
    if (i == 0) {
        KF_.filter(0.05, s, z_);
    } else {
        dt_ = (float)0.001*(data[i][0] - data[i-1][0]);
        KF_.filter(dt_, s, z_);
    }
  };

  void Navigation::StackingKFUpdate()
  {
    // TODO(george): Complete.
  };

  void Navigation::QueryKeyences()
  {
    // TODO(george): Complete.
  };

  bool Navigation::SensorsResultsSimilarity()
  {
    // TODO(george): Complete.
    return true;
  };

  void Navigation::timestampUpdate()
  {
    ImuDataPointArray sensorReadings = data_.getSensorsImuData();

    uint32_t t = sensorReadings.timestamp;
    dt_ = t - prev_timestamp_;
    prev_timestamp_ = t;
  }

  bool Navigation::calibrateGravity()
  {
    // TODO(george): Complete.
    return true;
  }

  // VectorXf Navigation::getState()
  // {
  //   return KF_.getState();
  // }

  void Navigation::run(int i)
  {
    // TODO(george): Complete.

    Navigation::IMUQuerying(i);

    // Stuff such as uncertainty and sensors disagreement will be added
  };

}}  // namespace hyped navigation
