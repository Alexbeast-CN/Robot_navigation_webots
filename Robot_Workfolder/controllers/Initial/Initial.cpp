// File:          Initial.cpp
// Date:          17th Nov 2021
// Description:   A project to build autonomous navigation robot
// Author:        Daoming Chen & Yifan Wang
// Modifications:

// STL
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <cmath>
#include <stack>
#include <queue>
#include <unordered_map>
#include <tuple>

// Webots Lib
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// Costumer Lib
#include "lib/SweepRobot.hpp"
#include "lib/Odometry.h"
#include "lib/Map.h"
#include "lib/Matrix.h"

// Environment variables
extern SweepRobot *SweepBot;
Map map;

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using std::cout;
using std::endl;
// Function prototypes:

/************************************* Main ********************************************/
int main(int argc, char **argv)
{
  // Load the map
  auto mat = map.easyMapS();
  // Display the map
  //mat.Show();
  // float left_pos;
  // float right_pos;

  Odometry Odo;
  float cor_x;
  float cor_y;
  float cor_theta;
  float t;

  //create the Robot instance.
  Robot *robot = new Robot();
  SweepBot = new SweepRobot(robot);
  // Set a speed for robot
  double Regular_speed = 40;
  
  // Main loop:
  while (robot->step(TIME_STEP) != -1)
  {
    t = robot->getTime();
    cout<<"Time is: " << t << endl;
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates();
    static int i=0;
    if ( i<10)
    {
      SweepBot->forward(Regular_speed);
      i++;
    }
    else
      {
        i=0;
        SweepBot-> turn_around_right(Regular_speed);
        SweepBot->stop();
      }
    robot->step(200);
  }
  

  
  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  return 0;
}
