/*
 * Author:
 * Organisation: HYPED
 * Date: 06/03/2020
 * Description: Outlier detection class
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
#include "navigation/outlier_detection.hpp"

namespace hyped {
using data::Data;
using data::DataPoint;
using data::ImuData;
using data::ModuleStatus;
using data::NavigationType;
using data::NavigationVector;
using utils::Logger;
using std::array;
using std::vector;

namespace navigation {


  int OutlierDetection::dead_IMUS()
  {
    int dead_imus = 0;
    // Detect dead IMUs (IMUs that have a zero reading)
    for (int i = 0; i < dataArray_.size(); i++) {
      if (dataArray_[i] == 0) {
        is_sensor_dead_[i] = false;
        dead_imus++;
      } else {
        is_sensor_dead_[i] = true;
      }
    }
    return dead_imus;
  }
  // Main outlier detection algorithm
  void OutlierDetection::detect_outliers()
  {
  NavigationType median = getMedian();
  NavigationType mean = getMean();
  NavigationType medAD = getMedianAD();
  NavigationType meanAD = getMeanAD(mean);
  Navigation::NavigationArray modZscore;
  for (int i = 0; i < dataArray_.size(); i++) {
    if (medAD != 0) {
      modZscore[i] = (dataArray_[i] - median) / (kMedianADCoeficient * medAD);
    } else {
      modZscore[i] = (dataArray_[i] - median) / (kMeanADCoeficient * meanAD);
    }
  }
  // Marks and replaces outliers with the median
  /*for (int i = 0; i < data ; i++) {
    if (fabs(modZscore[i]) > 3.5 || dataArray[i] == 0) {
      dataArray_[i] = median;
      imu_reliable_[i] = false;
    }
  }
  // Update imu_outlier_counter_ array (it is not used yet)
  for (int i = 0; i < dataArray_.size(); i++) {
    if (!imu_reliable_[i]) {
      imu_outlier_counter_[i]++;
    } else {
      imu_outlier_counter_[i] = 0;
    }
    */
  }
  NavigationType OutlierDetection::getMedian()
  {
    Navigation::NavigationArray dataArray_copy;
    std::copy(std::begin(dataArray_), std::end(dataArray_), std::begin(dataArray_copy));
    NavigationType median;
    NavigationType mid = dataArray_.size() / 2;

    // Calculate the median
    // This calculation is different in case half of sensors are faulty/dead (aka reading 0.0) since
    // that would break the algorithm.
    std::sort(std::begin(dataArray_copy), std::end(dataArray_copy));
    if (dead_IMUS() == data::Sensors::kNumImus / 2) {
        // Contains only non-zero readings of sensors to calculate a more realistic median, this is
        // due to the small number of sensors. This array is the same length as the original
        // (for consistency), does not contain faulty IMUs and duplicates the working ones.
        Navigation::NavigationArray filteredArray;
        int counter = 0;
        for (int i = 0; i < dataArray_.size(); i++) {
            if (is_sensor_dead_[i]) {
              filteredArray[counter] = dataArray_[i];
              filteredArray[counter + 1] = dataArray_[i];
              counter += 2;
            }
        }
        // Calculate the median using the filteredArray instead of dataArray_copy
        std::sort(std::begin(filteredArray), std::end(filteredArray));
        if (dataArray_.size() % 2 == 0) {
          median = (filteredArray[mid] + filteredArray[mid - 1]) / 2;
        } else {
            median = filteredArray[mid];
          }
    } else {
        // Regular median calculation
        if (dataArray_.size() % 2 == 0) {
          median = (dataArray_copy[mid] + dataArray_copy[mid - 1]) / 2;
      } else {
          median = dataArray_copy[mid];
      }
    }
    return median;
  }

  NavigationType OutlierDetection::getMean()
  {
    NavigationType mean = 0;
    for (int i = 0; i < dataArray_.size(); i++) {
      mean += dataArray_[i];
    }
    mean = mean / dataArray_.size();
    return mean;
  }
  NavigationType OutlierDetection::getMedianAD()
  {
    NavigationType medAD = 0;
    NavigationType median = 0;
    NavigationType mid = dataArray_.size() / 2;
    Navigation::NavigationArray medADarray;
    for (int i = 0; i < dataArray_.size(); i++) {
      medADarray[i] = fabs(dataArray_[i] - median);
    }
    std::sort(std::begin(medADarray), std::end(medADarray));
    if (dataArray_.size() % 2 == 0) {
      medAD = (medADarray[mid] + medADarray[mid - 1]) / 2;
    } else {
      medAD = medADarray[mid];
    }
    return medAD;
  }

  NavigationType OutlierDetection::getMeanAD(NavigationType mean)
  {
    NavigationType meanAD = 0;
    Navigation::NavigationArray meanADarray;
    for (int i = 0; i < dataArray_.size(); i++) {
      meanADarray[i] = fabs(dataArray_[i] - mean);
    }
    for (int i = 0; i < dataArray_.size(); i++) {
      meanAD += meanADarray[i];
    }
    meanAD = meanAD / dataArray_.size();
    return meanAD;
  }
}
}

