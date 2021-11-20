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

// Environment variables
SweepRobot *SweepBot;
// Odometry *location;

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Function prototypes:

/************************************* Main ********************************************/
int main(int argc, char **argv)
{
  float left_pos;
  float right_pos;
  float cor_x;
  float cor_y;
  float cor_theta;
  // create the Robot instance.
  Robot *robot = new Robot();
  SweepBot = new SweepRobot(robot);
  // Set a speed for robot
  double Regular_speed;
  Regular_speed = 20;
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  // create the Robot Odometry
  Odometry Odo;
  // Main loop:
  while (robot->step(TIME_STEP) != -1)
  {
    SweepBot->forward(Regular_speed);
    left_pos = SweepBot->leftposition();
    right_pos = SweepBot->rightposition();
    // std::cout<<"左轮的读数:"<<left_pos<<"  右轮的读数:"<<right_pos<<std::endl;
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates(left_pos, right_pos);
    std::cout<<"x坐标的读数:"<<cor_x<<std::endl;
    std::cout<<"y坐标的读数:"<<cor_y<<std::endl;
    std::cout<<"theta的读数:"<<cor_theta<<std::endl;
    //std::cout<<"y坐标的读数:"<<ve[1]<<std::endl;
    //std::cout<<"转动角度theta的读数:"<<ve[2]<<std::endl;
  }

  // Enter here exit cleanup code.

  delete robot;
  delete SweepBot;
  return 0;
}
