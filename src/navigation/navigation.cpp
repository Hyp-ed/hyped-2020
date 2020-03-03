/*
 * Author: Neil McBlane, Brano Pilnan, Justus Rudolph
 * Organisation: HYPED
 * Date: 16/02/2020
 * Description: Main file for navigation class.
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
#include <algorithm>

#include "navigation/navigation.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/timer.hpp"

namespace hyped {

using hyped::utils::concurrent::Thread;

namespace navigation {

Navigation::Navigation(Logger& log, unsigned int axis/*=0*/)
         : log_(log),
           data_(Data::getInstance()),
           status_(ModuleStatus::kStart),
           counter_(0),
           axis_(axis),
           calibration_limits_ {{0.05, 0.05, 0.05}},
           curr_msmt_(0),
           imu_reliable_ {{true, true, true, true}},
           nOutlierImus_(0),
           stripe_counter_(0, 0),
           keyence_used_(true),
           keyence_real_(true),
           keyence_failure_counter_(0),
           acceleration_(0, 0.),
           velocity_(0, 0.),
           distance_(0, 0.),
           distance_uncertainty_(0.),
           velocity_uncertainty_(0.),
           init_time_set_(false),
           acceleration_integrator_(&velocity_),
           velocity_integrator_(&distance_)
{
  log_.INFO("NAV", "Navigation module started");
  for (unsigned int i = 0; i < Sensors::kNumImus; i++) {
    filters_[i] = KalmanFilter(1, 1);
    filters_[i].setup();
  }
  status_ = ModuleStatus::kInit;
  updateData();
  log_.INFO("NAV", "Navigation module initialised");
}

ModuleStatus Navigation::getModuleStatus() const
{
  return status_;
}

// TODO(Neil/Lukas/Justus): do this more smartly?
NavigationType Navigation::getAcceleration() const
{
  return acceleration_.value;
}

// TODO(Neil/Lukas/Justus): do this more smartly?
NavigationType Navigation::getVelocity() const
{
  return velocity_.value;
}

// TODO(Neil/Lukas/Justus): do this more smartly?
NavigationType Navigation::getDistance() const
{
  return distance_.value;
}

NavigationType Navigation::getEmergencyBrakingDistance() const
{
  // TODO(Neil): Account for actuation delay and/or communication latency?
  return getVelocity()*getVelocity() / (2*kEmergencyDeceleration);
}

NavigationType Navigation::getBrakingDistance() const
{
  Motors motor_data = data_.getMotorData();
  uint32_t rpm = 0;
  for (int i = 0; i < data::Motors::kNumMotors; i++) {
    rpm += motor_data.rpms[i];
  }
  uint32_t avg_rpm = rpm / data::Motors::kNumMotors;
  float rot_velocity = (avg_rpm / 60) * (2 * pi);

  NavigationType actuation_force = spring_compression_ * spring_coefficient_;
  NavigationType braking_force = (actuation_force * coeff_friction_) /
                                 (tan(embrake_angle_) - coeff_friction_);
  NavigationType deceleration_total = kNumBrakes * braking_force / pod_mass_;

  NavigationType pod_kinetic_energy = 0.5 * pod_mass_ * getVelocity() * getVelocity();
  NavigationType rotational_kinetic_energy = data::Motors::kNumMotors * 0.5 * mom_inertia_wheel_ *
                                             rot_velocity * rot_velocity;
  NavigationType total_kinetic_energy = pod_kinetic_energy + rotational_kinetic_energy;

  NavigationType braking_distance = (total_kinetic_energy / pod_mass_) / deceleration_total;

  return braking_distance;
}

Navigation::NavigationVectorArray Navigation::getGravityCalibration() const
{
  return gravity_calibration_;
}

void Navigation::calibrateGravity()
{
  log_.INFO("NAV", "Calibrating gravity");
  array<RollingStatistics<NavigationVector>, Sensors::kNumImus> online_array =
    {{ RollingStatistics<NavigationVector>(kCalibrationQueries),
       RollingStatistics<NavigationVector>(kCalibrationQueries),
       RollingStatistics<NavigationVector>(kCalibrationQueries),
       RollingStatistics<NavigationVector>(kCalibrationQueries) }};
  bool calibration_successful = false;
  int calibration_attempts = 0;

  while (!calibration_successful && calibration_attempts < kCalibrationAttempts) {
    log_.INFO("NAV", "Calibration attempt %d", calibration_attempts+1);

    // Average each sensor over specified number of readings
    for (int i = 0; i < kCalibrationQueries; ++i) {
      sensor_readings_ = data_.getSensorsImuData();
      for (int j = 0; j < Sensors::kNumImus; ++j) {
        online_array[j].update(sensor_readings_.value[j].acc);
      }
      Thread::sleep(1);
    }
    // Check if each calibration's variance is acceptable
    calibration_successful = true;
    for (int i = 0; i < Sensors::kNumImus; ++i) {
      for (int j = 0; j < 3; ++j) {
        bool var_within_lim = online_array[i].getVariance()[j] < calibration_limits_[j];
        calibration_successful = calibration_successful && var_within_lim;
      }
    }
    calibration_attempts++;
  }

  // Store calibration and update filters if successful
  if (calibration_successful) {
    log_.INFO("NAV", "Calibration of IMU acceleration succeeded with final readings:");
    for (int i = 0; i < Sensors::kNumImus; ++i) {
      gravity_calibration_[i] = online_array[i].getMean();
      double var = 0.0;
      for (int j = 0; j < 3; ++j) {
        var += online_array[i].getVariance()[j];
      }
      filters_[i].updateMeasurementCovarianceMatrix(var);

      log_.INFO("NAV", "\tIMU %d: g=(%.5f, %.5f, %.5f), var=%.5f",
              i, gravity_calibration_[i][0], gravity_calibration_[i][1],
              gravity_calibration_[i][2], var);
    }
    // set calibration uncertainties
    for (int axis = 0; axis < 3; axis++) {
      for (int i = 0; i < Sensors::kNumImus; i++) {
        double var = (online_array[i].getVariance()[axis]);
        calibration_variance_[axis] += var*var;
      }
      // geometric mean for variances of different IMUs
      calibration_variance_[axis] = sqrt(calibration_variance_[axis]);
    }
    log_.INFO("NAV", "Calibration Variance: x-axis: %.3f, y-axis: %.3f, z-axis: %.3f",
      calibration_variance_[0], calibration_variance_[1], calibration_variance_[2]);
    status_ = ModuleStatus::kReady;
    updateData();
    log_.INFO("NAV", "Navigation module ready");
  } else {
    log_.INFO("NAV", "Calibration of IMU acceleration failed with final readings:");
    for (int i = 0; i < Sensors::kNumImus; ++i) {
      NavigationVector acc = online_array[i].getMean();
      double var = 0.0;
      for (int j = 0; j < 3; ++j) {
        var += online_array[i].getVariance()[j];
      }

      log_.INFO("NAV", "\tIMU %d: g=(%.5f, %.5f, %.5f), var=%.5f", i, acc[0], acc[1], acc[2], var);
    }
    status_ = ModuleStatus::kCriticalFailure;
    updateData();
    log_.ERR("NAV", "Navigation module failed on calibration");
  }
}

NavigationType Navigation::accNorm(NavigationVector& acc)
{
  NavigationType norm = 0.0;
  for (unsigned int i = 0; i < 3; i ++) {
      NavigationType a = acc[i];
      norm += a*a;
  }
  norm = sqrt(norm);
  return norm;
}

void Navigation::queryImus()
{
  ImuAxisData acc_raw;  // All raw data, four values per axis
  NavigationArray acc_raw_moving;  // Raw values in moving axis
  OnlineStatistics<NavigationType> acc_avg_filter;
  sensor_readings_ = data_.getSensorsImuData();
  uint32_t t = sensor_readings_.timestamp;
  // process raw values
  for (uint8_t axis = 0; axis < 3; axis++) {
    for (int i = 0; i < Sensors::kNumImus; ++i) {
      NavigationVector a = sensor_readings_.value[i].acc - gravity_calibration_[i];
      acc_raw[axis][i] = a[axis];

      // the moving axis should be set to 0 for tukeyFences
      if (!imu_reliable_[i]) {
        acc_raw_moving[i] = 0;
      } else if (axis == axis_) {
        acc_raw_moving[i] = a[axis_];
      }
    }
  }
  log_.DBG1("NAV", "Raw acceleration values: %.3f, %.3f, %.3f, %.3f", acc_raw_moving[0],
            acc_raw_moving[1], acc_raw_moving[2], acc_raw_moving[3]);
  // Run outlier detection on moving axis
  // tukeyFences(acc_raw_moving, kTukeyThreshold);
  m_zscore(acc_raw_moving);
  // TODO(Justus) how to run outlier detection on non-moving axes without affecting "reliable"
  // Current idea: outlier function takes reliability write flag, on hold until z-score impl.

  /*for (int axis = 0; axis < 3; axis++) {
    if (axis != axis_) tukeyFences(acc_raw[axis], kTukeyThreshold);
  }*/

  // Kalman filter the readings which are reliable
  for (int i = 0; i < Sensors::kNumImus; ++i) {
    if (imu_reliable_[i]) {
      NavigationType estimate = filters_[i].filter(acc_raw_moving[i]);
      acc_avg_filter.update(estimate);
    }
  }
  previous_measurements_[curr_msmt_] = acc_raw;
  curr_msmt_++;
  if (curr_msmt_ == kPreviousMeasurements) {
    curr_msmt_ = 0;
    prev_filled_ = 1;
  }
  if (prev_filled_) checkVibration();

  acceleration_.value = acc_avg_filter.getMean();
  acceleration_.timestamp = t;

  acceleration_integrator_.update(acceleration_);
  velocity_integrator_.update(velocity_);
}

void Navigation::checkVibration()
{
  // curr_msmt points at next measurement, ie the last one
  array<OnlineStatistics<NavigationType>, 3> online_array_axis;
  for (int i = 0; i < kPreviousMeasurements; i++) {
    ImuAxisData raw_data = previous_measurements_[(curr_msmt_ + i) % kPreviousMeasurements];
    for (uint8_t axis = 0; axis < 3; axis++) {
      if (axis != axis_) {  // assume variance in moving axis are not vibrations
        for (int imu = 0; imu < Sensors::kNumImus; imu++) {
          online_array_axis[axis].update(raw_data[axis][imu]);
        }
      }
    }
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    double var = online_array_axis[axis].getVariance();
    if (counter_ % 100000 == 0 && axis != axis_) {
      log_.INFO("NAV", "Variance in axis %d: %.3f", axis, var);
    }
    double ratio = var / calibration_variance_[axis];
    double statistical_variance_ratio = kCalibrationQueries/kPreviousMeasurements;
    if (ratio > statistical_variance_ratio) {
      log_.ERR("NAV", "Variance in axis %d is %.3f times larger than its calibration variance.",
        axis, ratio);
    }
  }
}

void Navigation::updateUncertainty()
{
  /* Uncertainty from measuring is the timestamp between two measurements times velocity.
   * Furthermore, the velocity has an uncertainty due to acceleration and timestamp. */
  double delta_t = (distance_.timestamp - prev_timestamp_)/1000000.0;
  NavigationType abs_delta_acc = abs(getAcceleration() - prev_acc_);
  // Adds uncertainty from the possible shift in both directions in the timestamp
  velocity_uncertainty_ += abs_delta_acc*delta_t/2.;
  // Adds uncertainty from the variance in acceleration from measurements
  NavigationType acc_variance = 0.0;
  for (int i = 0; i < Sensors::kNumImus; i++) {
    acc_variance += filters_[i].KalmanFilter::getEstimateVariance();
  }
  // Average variance
  acc_variance = acc_variance/Sensors::kNumImus;
  // Standard deviation
  NavigationType acc_stdDev = sqrt(acc_variance);
  // uncertainty is the std deviation integrated to give velocity
  velocity_uncertainty_ += acc_stdDev*delta_t;
  // Hence uncertainty in distance becomes updated with:
  distance_uncertainty_ += velocity_uncertainty_*delta_t;
  // Also, distance will be affected by taking the average of two velocities
  distance_uncertainty_ += abs(getVelocity() - prev_vel_) * delta_t / 2.;
}

void Navigation::queryKeyence()
{
  // set the keyence readings with the data from the central data struct
  keyence_readings_ = data_.getSensorsKeyenceData();
  for (int i = 0; i < Sensors::kNumKeyence; i++) {
    // Checks whether the stripe count has been updated and if it has not been
    // double-counted with the time constraint (100000micros atm, aka 0.1s, subject to change).
    if (prev_keyence_readings_[i].count.value != keyence_readings_[i].count.value &&
         keyence_readings_[i].count.timestamp - stripe_counter_.timestamp > 1e5) {
      stripe_counter_.value++;
      stripe_counter_.timestamp = keyence_readings_[i].count.timestamp;
      if (!keyence_real_) stripe_counter_.timestamp = utils::Timer::getTimeMicros();

      // Allow up to one missed stripe.
      // There must be some uncertainty in distance around the missed 30.48m (kStripeDistance).
      NavigationType allowed_uncertainty = distance_uncertainty_;
      /* If the uncertainty is too small, it is set to a relatively small value so that we do
       * not get an error just because the uncertainty is tiny. */
      NavigationType minimum_uncertainty = kStripeDistance / 5.;
      if (distance_uncertainty_ < minimum_uncertainty) allowed_uncertainty = minimum_uncertainty;
      NavigationType distance_change = distance_.value - stripe_counter_.value*kStripeDistance;
      /* There should only be an updated stripe count if the IMU determined distance is closer
       * to the the next stripe than the current. It should not just lie within the uncertainty,
       * otherwise we might count way more stripes than there are as soon as the uncertainty gets
       * fairly large (>15m). */
      if (distance_change > kStripeDistance - allowed_uncertainty &&
          distance_change < kStripeDistance + allowed_uncertainty &&
          distance_.value > stripe_counter_.value*kStripeDistance + 0.5*kStripeDistance) {
        stripe_counter_.value++;
        distance_change -= kStripeDistance;
      }
      /* Error handling: If distance from keyence still deviates more than the allowed
      uncertainty, then the measurements are no longer believable. Important that this
      is only checked in an update, otherwise we might throw an error in between stripes.
      The first stripe is very uncertain, since it takes the longest, thus we ignore it.
      Even if the first stripe is missed, error handling will catch it when the second is seen.*/
      if ((distance_change < (-2) * allowed_uncertainty) ||
          (distance_change > 2 * allowed_uncertainty))
      {
        keyence_failure_counter_++;
        keyence_failure_counter_ += floor(abs(distance_change) / kStripeDistance);
      }
      // Lower the uncertainty in velocity (based on sinuisoidal distribution):
      velocity_uncertainty_ -= abs(distance_change*1e6/
                               (stripe_counter_.timestamp - init_timestamp_));
      log_.DBG("NAV", "Stripe detected!");
      log_.DBG1("NAV", "Timestamp difference: %d", stripe_counter_.timestamp - init_timestamp_);
      log_.DBG1("NAV", "Timestamp currently:  %d", stripe_counter_.timestamp);

      // Make sure velocity uncertainty is positive.
      velocity_uncertainty_ = abs(velocity_uncertainty_);
      // The uncertainty in distance is not changed from this because the impact is far too large
      // Update velocity value
      velocity_.value -= distance_change*1e6/(stripe_counter_.timestamp - init_timestamp_);
      // Update distance value
      distance_.value -= distance_change;
      break;
    }
  }
  // If more than one disagreement occurs then we enter the kCriticalFailure state
  if (keyence_failure_counter_ > 1) {
    status_ = ModuleStatus::kCriticalFailure;
    log_.ERR("NAV", "More than one large IMU/Keyence disagreement, entering kCriticalFailure");
  }
  /* Similarly, if the current IMU distance is larger than four times the distance between
   * two stripes, then we know that the two can no longer agree. That is because at least
   * three stripes have been missed then, which throws kCriticalFailure. */
  if (distance_.value - stripe_counter_.value*kStripeDistance > 4 * kStripeDistance) {
    status_ = ModuleStatus::kCriticalFailure;
    log_.ERR("NAV", "IMU distance at least 3 * kStripeDistance ahead, entering kCriticalFailure.");
  }
  // Update old keyence readings with current ones
  prev_keyence_readings_ = keyence_readings_;
}

void Navigation::disableKeyenceUsage()
{
  keyence_used_ = false;
}

void Navigation::setKeyenceFake()
{
  keyence_real_ = false;
}

bool Navigation::getHasInit()
{
  return init_time_set_;
}

void Navigation::setHasInit()
{
  init_time_set_ = true;
}

template <class OutlierType>
void Navigation::m_zscore(OutlierType& data_array)
{
  OutlierType data_array_copy;
  std::copy(std::begin(data_array), std::end(data_array), std::begin(data_array_copy));
  const int length    = data_array.size();
  int mid             = length / 2;
  int dead_imus = 0;
  float median        = 0;
  float mean          = 0;
  float medAD         = 0;
  float meanAD        = 0;

  // Detect dead IMUs (IMUs that have a zero reading)
  for (int i = 0; i < length; i++) {
    if (data_array[i] == 0) {
      imu_reliable_[i] = false;
      dead_imus++;
    } else {
      imu_reliable_[i] = true;
    }
  }
  // Calculate the median
  // This calculation is different in case half of sensors are faulty/dead (aka reading 0.0) since
  // that would break the algorithm.
  std::sort(std::begin(data_array_copy), std::end(data_array_copy));
  if (dead_imus == data::Sensors::kNumImus / 2) {
      // Contains only non-zero readings of sensors to calculate a more realistic median, this is
      // due to the small number of sensors. This array is the same length as the original
      // (for consistency), does not contain faulty IMUs and duplicates the working ones.
      OutlierType filtered_array;
      int counter = 0;
      for (int i = 0; i < length; i++) {
          if (imu_reliable_[i]) {
            filtered_array[counter] = data_array[i];
            filtered_array[counter + 1] = data_array[i];
            counter += 2;
          }
      }
      // Calculate the median using the filtered_array instead of data_array_copy
      std::sort(std::begin(filtered_array), std::end(filtered_array));
      if (length % 2 == 0) {
         median = (filtered_array[mid] + filtered_array[mid - 1]) / 2;
      } else {
           median = filtered_array[mid];
        }
  } else {
      // Regular median calculation
      if (length % 2 == 0) {
        median = (data_array_copy[mid] + data_array_copy[mid - 1]) / 2;
    } else {
        median = data_array_copy[mid];
    }
  }
  // Calculate the mean value
  for (int i = 0; i < length; i++) {
    mean += data_array_copy[i];
  }
  mean = mean / length;
  // Calculate the absolute difference of each value from the median
  OutlierType medADarray;
  for (int i = 0; i < length; i++) {
    medADarray[i] = fabs(data_array[i] - median);
  }
  // Calculate the median of that, aka the Median Absolute Deviation
  std::sort(std::begin(medADarray), std::end(medADarray));
  if (length % 2 == 0) {
    medAD = (medADarray[mid] + medADarray[mid - 1]) / 2;
  } else {
    medAD = medADarray[mid];
  }
  // Calculate the absolute difference of each value from the mean
  OutlierType meanADarray;
  for (int i = 0; i < length; i++) {
    meanADarray[i] = fabs(data_array[i] - mean);
  }
  // Calculate the mean of that, aka the Mean Absolute Deviation
  for (int i = 0; i < length; i++) {
    meanAD += meanADarray[i];
  }
  meanAD = meanAD / length;
  // Calculate the Z-score of each value
  OutlierType modZscore;
  for (int i = 0; i < length; i++) {
    if (medAD != 0) {
      modZscore[i] = (data_array[i] - median) / (kMedianADCoeficient * medAD);
    } else {
      modZscore[i] = (data_array[i] - median) / (kMeanADCoeficient * meanAD);
    }
  }
  // Marks and replaces outliers with the median
  for (int i = 0; i < length ; i++) {
    if (fabs(modZscore[i]) > 3.5 || data_array[i] == 0) {
      data_array[i] = median;
      imu_reliable_[i] = false;
    }
  }
  // Update imu_outlier_counter_ array (it is not used yet)
  for (int i = 0; i < length; i++) {
    if (!imu_reliable_[i]) {
      imu_outlier_counter_[i]++;
    } else {
      imu_outlier_counter_[i] = 0;
    }
  }
}

void Navigation::updateData()
{
  data::Navigation nav_data;
  nav_data.module_status              = getModuleStatus();
  nav_data.distance                   = getDistance();
  nav_data.velocity                   = getVelocity();
  nav_data.acceleration               = getAcceleration();
  nav_data.emergency_braking_distance = getEmergencyBrakingDistance();
  nav_data.braking_distance           = 1.2 * getEmergencyBrakingDistance();

  data_.setNavigationData(nav_data);

  if (counter_ % 100 == 0) {  // kPrintFreq
    log_.DBG("NAV", "%d: Data Update: a=%.3f, v=%.3f, d=%.3f, d(keyence)=%.3f", //NOLINT
               counter_, nav_data.acceleration, nav_data.velocity, nav_data.distance,
               stripe_counter_.value*kStripeDistance);
    log_.DBG("NAV", "%d: Data Update: v(unc)=%.3f, d(unc)=%.3f, keyence failures: %d",
               counter_, velocity_uncertainty_, distance_uncertainty_, keyence_failure_counter_);
  }
  counter_++;
  // Update all prev measurements
  prev_timestamp_ = distance_.timestamp;
  prev_acc_ = getAcceleration();
  prev_vel_ = getVelocity();
}

void Navigation::navigate()
{
  queryImus();
  if (keyence_used_) queryKeyence();
  if (counter_ > 1000) updateUncertainty();
  updateData();
}

void Navigation::initTimestamps()
{
  // First iteration --> set timestamps
  acceleration_.timestamp = utils::Timer::getTimeMicros();
  velocity_    .timestamp = utils::Timer::getTimeMicros();
  distance_    .timestamp = utils::Timer::getTimeMicros();
  prev_acc_ = getAcceleration();
  prev_vel_ = getVelocity();
  init_timestamp_ = utils::Timer::getTimeMicros();
  log_.DBG3("NAV", "Initial timestamp:%d", init_timestamp_);
  prev_timestamp_ = utils::Timer::getTimeMicros();
  // First iteration --> get initial keyence data
  // (should be zero counters and corresponding timestamp)
  prev_keyence_readings_ = data_.getSensorsKeyenceData();
}
}}  // namespace hyped::navigation
