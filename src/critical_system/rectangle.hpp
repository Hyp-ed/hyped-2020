/*
* Author: QA team
* Organisation: HYPED
* Date:
* Description:
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

#ifndef CRITICAL_SYSTEM_RECTANGLE_HPP_
#define CRITICAL_SYSTEM_RECTANGLE_HPP_

namespace hyped {
namespace critical_system {
  class Rectangle {
  public:
    Rectangle();
    Rectangle(int height, int width);

    int area();

    int getWidth();

    void setWidth(int width);

    int getHeight();

    void setHeight(int height);
  private:
    int width_, height_;
  };
}
}
#endif  // CRITICAL_SYSTEM_RECTANGLE_HPP_
