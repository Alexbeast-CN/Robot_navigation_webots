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
  Regular_speed = 50;
  float t1 = robot->getTime();
  cout<<t1<<endl;
  SweepBot->forward(Regular_speed);
  SweepBot->delay_ms(1000);
  t1 = robot->getTime();
  cout<<t1<<endl;

  SweepBot->turn_around_right(Regular_speed);
  t1 = robot->getTime();
  cout<<t1<<endl;

  /*
  // Main loop:
  while (robot->step(TIME_STEP) != -1)
  {
    left_pos = SweepBot->leftposition();
    right_pos = SweepBot->rightposition();
    // std::cout<<"左轮的读数:"<<left_pos<<"  右轮的读数:"<<right_pos<<std::endl;
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates(left_pos, right_pos);

    //std::cout<<"y坐标的读数:"<<ve[1]<<std::endl;
    //std::cout<<"转动角度theta的读数:"<<ve[2]<<std::endl;
    
    // The inital place of the robot on the map
    int map_x = cor_x*10 + 1;
    int map_y = cor_y*10 + 1;

    // The coordinate that is 2 cells ahead of the robot
    int ahead_x = map_x + 3*cos(abs(cor_theta));
    int ahead_y = map_y + 3*sin(abs(cor_theta));
    cout << "Ahead x is " << ahead_x << ", ahead y is " << ahead_y << endl;
    // // Motion logic
    // if (mat.Point(ahead_x,ahead_y)==1&&cor_theta<1)
    //   SweepBot->turn_around_right(Regular_speed);
    // else if (mat.Point(ahead_x,ahead_y)==1&&cor_theta>3)
    //   SweepBot->turn_around_left(Regular_speed);
    // else 
    //   SweepBot->forward(Regular_speed);
  }
    */

  // Enter exit cleanup code here.

  delete robot;
  delete SweepBot;
  return 0;
}
