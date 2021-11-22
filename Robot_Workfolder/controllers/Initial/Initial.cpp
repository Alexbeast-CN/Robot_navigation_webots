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
  auto mat = map.easyMapS();
  // Display the map
  //mat.Show();
  float left_pos;
  float right_pos;

  Odometry Odo;
  float cor_x;
  float cor_y;
  float cor_theta;
  float t;
  //create the Robot instance.
  Robot *robot = new Robot();
  SweepBot = new SweepRobot(robot);
  // Set a speed for robot
  double Regular_speed;
  Regular_speed = 40;
  int map_x;
  int map_y;
  int ahead_x;
  int ahead_y;
  int x_left;
  int x_right;
  int y_left;
  int y_right;

  // Main loop:
  while (robot->step(TIME_STEP) != -1)
  {
    t = robot->getTime();
    cout<<"Time is: " << t << endl;
    left_pos = SweepBot->leftposition();
    right_pos = SweepBot->rightposition();
    // std::cout<<"左轮的读数:"<<left_pos<<"  右轮的读数:"<<right_pos<<std::endl;
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates(left_pos, right_pos);
    
    // The inital place of the robot on the map
    map_x = cor_x*10 + 1;
    map_y = cor_y*10 + 1;

    // The coordinate that is 2 cells ahead of the robot
    ahead_x = map_x + 2*cos(abs(cor_theta));
    ahead_y = map_y + 2*sin(abs(cor_theta));
    x_left = map_x - 1*sin(abs(cor_theta));
    x_right = map_x + 1*sin(abs(cor_theta));
    y_left = map_y - 1*cos(abs(cor_theta));
    y_right = map_y + 1*cos(abs(cor_theta));
    cout << "Ahead x is " << ahead_x << ", ahead y is " << ahead_y << endl;

    // // Motion logic
    if (mat.Point(ahead_x,ahead_y)==1)
    {
      if (mat.Point(map_x,y_right)==1)
      {
        if (mat.Point(map_x,y_left)==1)
          SweepBot->stop();
        else
          SweepBot->turn_around_left(Regular_speed);
      }
      else if (mat.Point(map_x,y_left)==1)
      {
        if (mat.Point(map_x,y_right)==1)
          SweepBot->stop();
        else
          SweepBot->turn_around_right(Regular_speed);
      }
      else 
        SweepBot->turn_around_right(Regular_speed);
    }
    else
      SweepBot->forward(Regular_speed);
    
    robot->step(200);
  }
  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  return 0;
}
