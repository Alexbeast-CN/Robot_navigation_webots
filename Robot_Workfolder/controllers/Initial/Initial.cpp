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
#include <map>
#include <queue>
#include <cstdio>
#include <utility>

// Webots Lib
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// Costumer Lib
#include "lib/SweepRobot.hpp"
#include "lib/Odometry.h"
#include "lib/Map.h"
#include "lib/Matrix.h"
#include "lib/Astar.h"

// Environment variables
#define PI4Turn 3.12
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
  Matrix mat = map.easyMapS();
  // Display the map
  //mat.Show();

  Odometry Odo;
  float cor_x;
  float cor_y;
  float cor_theta;
  float Initial_theta = 0.0;
  float t;
  
  //A星算法测试(函数可以返回整个容器，并且关联关系是没问题的)
  Astar Path;
  std::map<std::pair<int,int>,std::pair<int,int> >Route;
  std::pair<int,int>aa;
  // 假设起始点和终点已知
  int a[2] = {1,1};
  int b[2] = {1,4};
  std::pair<int,int>end(1,2);
  Route = Path.Findpath(a,b,mat);
  aa = Route[end];
  cout<<aa.first<<' '<<aa.second<<endl;

  //create the Robot instance.
  Robot *robot = new Robot();
  SweepBot = new SweepRobot(robot);
  // Set a speed for robot
  double Regular_speed = 40;
  
  // Main loop:
  while (robot->step(TIME_STEP) != -1)
  {
    // Broadcast the robot's pose.
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates();

    // Motion test
    static int i=0;
    if (i<10)
    {
      SweepBot->forward(Regular_speed);
      i++;
    }
    else if (cor_theta > Initial_theta + PI4Turn)
      SweepBot->stop();
    else
    {
      SweepBot-> turn_around_right(Regular_speed);
    }

    // Broadcast the timestamp
    t = robot->getTime();
    cout<<"Time is: " << t << endl;
  }
  

  
  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  return 0;
}
