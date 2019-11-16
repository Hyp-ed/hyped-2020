/*
 * Author: George Karabassis
 * Organisation: HYPED
 * Date: 30/03/2019
 * Description: Fake IMU data generator testing
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


#include "IMUs_faker.hpp"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::VectorXf;
using Eigen::MatrixXf;

demo_IMU_data::demo_IMU_data(string filename, int m)
{
    file_name_  = filename;
    m_          = m;
}

vector< vector<float> > demo_IMU_data::get_data()
{
    vector< vector<float> > data;

    string line;
    ifstream myfile (file_name_);
    int i = 0;

    if (myfile.is_open())
    {
        vector<float> dt_acc;
        dt_acc.push_back(0.0);
        for(int d = 0; d < m_; d++){
            dt_acc.push_back(0.0);
        }
        
        while (! myfile.eof() )
        {

            getline (myfile,line, '\t');
            dt_acc.at(0) = strtof((line).c_str(),0); //string to flaot

            for(int d = 0; d < m_; d++)
            {
                if (d == m_ -1){
                    getline (myfile,line, '\n');
                }
                else{
                    getline (myfile,line, '\t');
                }
                dt_acc.at(1+d) = strtof((line).c_str(),0); //string to flaot
            }

            data.push_back(dt_acc);

            //resetting touple
            dt_acc.at(0) = 0;
            for(int d = 0; d < m_; d++)
            {
                dt_acc.at(1+d) = 0;
            }
                
        }
        myfile.close();
    }
    else cout << "Unable to open file";

    // vector<array<double, 2> > data = {{1.0,2.0},{3.0,4.0}};

    return data;
    
}