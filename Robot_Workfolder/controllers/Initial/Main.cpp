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

// Webots Lib
#include <webots/Robot.hpp>

// Costumer Lib
#include "lib/Robot.hpp"

// Environment variables
SweepRobot *SweepBot;

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Function prototypes:


/************************************* Main ********************************************/
int main(int argc, char **argv) {

  // create the Robot instance.
  Robot *robot = new Robot();
  SweepBot = new SweepRobot(robot);

  // Set a speed for robot
  double Regular_speed;
  Regular_speed = 20;

  // Main loop:
  while (robot->step(TIME_STEP) != -1) 
  {
    SweepBot->forward(Regular_speed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}