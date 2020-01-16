/*
 * Authors: Calum McMeekin
 * Organisation: HYPED
 * Date:
 * Description:
 * HypedMachine wraps around State objects, i.e. functions as a State manager.
 * HypedMachine reacts to events through handleEvent() function. Fucntion transition()
 * finalises changes of current state and facilitate updates to the shared data structure.
 *
 *    Copyright 2019 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef CRITICALSYSTEMS_RECTANGLE_HPP_
#define CRITICALSYSTEMS_RECTANGLE_HPP_

class Rectangle
{
  private:
    int width;
    int height;

  public:
    Rectangle();
    virtual  void changeWidth(int w);
    virtual  void changeHeight(int h);
    virtual  int getWidth() {return width; }
    virtual  int getHeight() {return height; }
    virtual  int area() {return width*height; }
};

#endif  // CRITICALSYSTEMS_RECTANGLE_HPP_
