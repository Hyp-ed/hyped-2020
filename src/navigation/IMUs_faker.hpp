/*
 * Author: George Karabassis
 * Organisation: HYPED
 * Date: 30/03/2019
 * Description: Fake IMU data generator testing (template)
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

#ifndef NAVIGATION_IMUS_FAKER_HPP_
#define NAVIGATION_IMUS_FAKER_HPP_

#include <iostream>
#include <vector>
#include <string>
#include "../lib/Eigen/Dense"

using namespace std;

namespace hyped {
namespace navigation {

class demoIMUData {
    public:
        demoIMUData(string file_name, int m);
        vector< vector<float> > getData();

    public:
        string file_name_;
        int m_;
};

}}  // namespace hyped::navigation

#endif  // NAVIGATION_IMUS_FAKER_HPP_
