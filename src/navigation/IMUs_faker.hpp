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

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class demo_IMU_data{

    public:

        demo_IMU_data(string file_name, int m);

        vector< vector<float> > get_data();

    public:

        string file_name_;
        int m_;
};
