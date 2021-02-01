
/*
 * Author: Florica Margaritescu
 * Organisation: HYPED
 * Date: 15/01/2021
 * Description: Testing functionality for vector.hpp (float elements)
 *
 *    Copyright 2020 HYPED
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

#include <math.h>
#include <cstdlib>
#include "gtest/gtest.h"
#include "utils/math/vector.hpp"

namespace hyped {
namespace utils {
namespace math {

// -------------------------------------------------------------------------------------------------
// You can define helper functions - for checking the norm() function for example
// -------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------
// Set Up
// -------------------------------------------------------------------------------------------------

/**
* @brief Struct used for test fixtures checking class functionality and properties.
*/
struct vector_test_workshop : public ::testing::Test 
{
   protected:
    // DECLARATIONS GO HERE
    // For example, create a vector and an array to construct the vector
    Vector<int, 100> test_vector1_int;
    std::array<int, 100> elements1;


    // DEFINITIONS GO HERE
    void SetUp() 
    {
       // Generate array of random integer values
        for (int i = 0; i < elements1.size(); i++) {
           elements1[i] = rand() % 1000 + 1;  // random values form 1 to 1000 
        }

        // Construct vector 
        test_vector1_int =  Vector<int, 100>(elements1);
    }

  void TearDown() {}
}; 

// -------------------------------------------------------------------------------------------------
// TEST EXAMPLE
// -------------------------------------------------------------------------------------------------

/**
* @brief Tests constructor given an array
*/
TEST_F(vector_test_workshop, testVectorConstructorArrayWorkshop) 
{ // Checking if elmements in array at given position match vector elements 
  // at given position
  for (int i = 0; i < elements1.size(); i++) {
     ASSERT_EQ(elements1[i], test_vector1_int.operator[](i));
  }
}

}}}  // hyped::utils::math
