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
SweepRobot *SweepBot;
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
  auto mat = map.easyMap();
  // Display the map
  //mat.Show();

  Odometry Odo;
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


  // Main loop:
  while (robot->step(TIME_STEP) != -1)
  {
    SweepBot->forward(Regular_speed);
    left_pos = SweepBot->leftposition();
    right_pos = SweepBot->rightposition();
    // std::cout<<"左轮的读数:"<<left_pos<<"  右轮的读数:"<<right_pos<<std::endl;
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates(left_pos, right_pos);

    //std::cout<<"y坐标的读数:"<<ve[1]<<std::endl;
    //std::cout<<"转动角度theta的读数:"<<ve[2]<<std::endl;

    // The inital place of the robot on the map
    int map_x = cor_x*100 + 5;
    int map_y = cor_y*100 + 5;

    // The coordinate that is 10 cells ahead of the robot
    int ahead_x = map_x + 10*cos(abs(cor_theta));
    int ahead_y = map_y + 10*sin(abs(cor_theta));

    // Motion logic
    if (mat.Point(ahead_x,ahead_y)==1&&cor_theta>=0)
      SweepBot->turn_around_right(Regular_speed);
    if (mat.Point(ahead_x,ahead_y)==1&&cor_theta<3)
      SweepBot->turn_around_left(Regular_speed);
    

  }

  // Enter here exit cleanup code.

  delete robot;
  delete SweepBot;
  return 0;
}
