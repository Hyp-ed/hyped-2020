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

  NavigationType OutlierDetection::getMedian()
  {
    Navigation::NavigationArray data_array_copy;
    std::copy(std::begin(dataArray_), std::end(dataArray_), std::begin(data_array_copy));
    NavigationType median;
    NavigationType mid = dataArray_.size() / 2;

    // Calculate the median
    // This calculation is different in case half of sensors are faulty/dead (aka reading 0.0) since
    // that would break the algorithm.
    std::sort(std::begin(data_array_copy), std::end(data_array_copy));
    if (dead_IMUS() == data::Sensors::kNumImus / 2) {
        // Contains only non-zero readings of sensors to calculate a more realistic median, this is
        // due to the small number of sensors. This array is the same length as the original
        // (for consistency), does not contain faulty IMUs and duplicates the working ones.
        Navigation::NavigationArray filtered_array;
        int counter = 0;
        for (int i = 0; i < dataArray_.size(); i++) {
            if (is_sensor_dead_[i]) {
              filtered_array[counter] = dataArray_[i];
              filtered_array[counter + 1] = dataArray_[i];
              counter += 2;
            }
        }
        // Calculate the median using the filtered_array instead of data_array_copy
        std::sort(std::begin(filtered_array), std::end(filtered_array));
        if (dataArray_.size() % 2 == 0) {
          median = (filtered_array[mid] + filtered_array[mid - 1]) / 2;
        } else {
            median = filtered_array[mid];
          }
    } else {
        // Regular median calculation
        if (dataArray_.size() % 2 == 0) {
          median = (data_array_copy[mid] + data_array_copy[mid - 1]) / 2;
      } else {
          median = data_array_copy[mid];
      }
    }
    return median;
  }
}
}

