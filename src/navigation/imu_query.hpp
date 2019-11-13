#include <unistd.h>
#include <cstdint>
#include <string>

#include <cstdio>
#include <fstream>
#include <Eigen/Dense>

#include "data/data.hpp"
#include "data/data_point.hpp"

using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace hyped {

namespace navigation {

class IMU_quering{

  public: 
  
    IMU_query(Data data_);

    void data_formatting();

    void outlier_check();

    VectorXf result();

};

}}