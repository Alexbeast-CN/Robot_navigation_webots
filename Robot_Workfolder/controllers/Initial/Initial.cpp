// File:          Initial.cpp
// Date:          17th Nov 2021
// Description:   A project to build autonomous navigation robot
// Author:        Daoming Chen & Yifan Wang
// Modifications:

// Costumer Lib
#include "lib/SweepRobot.hpp"
#include "lib/Odometry.h"
#include "lib/Map.h"
#include "lib/Matrix.h"
#include "lib/Astar.h"

// Environment variables
#define PI4Turn 3.14
SweepRobot *SweepBot;
Map map;
double Regular_speed = 40;

// Load the map
Matrix mat = map.easyMapS();
// Display the map
//mat.Show();
int map_x;
int map_y;
float map_theta;

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using std::cout;
using std::endl;

// Function prototypes:
int easyBPP();

// State define
int state;
#define BPP   1
#define TURNL 2
#define TURNR 3

/************************************* Main ********************************************/
int main(int argc, char **argv)
{
  state = BPP;

  Odometry Odo;
  float cor_x;
  float cor_y;
  float cor_theta;
  float Initial_theta = 0.0;
  // float t;
  
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

  // Main loop:
  while (robot->step(TIME_STEP) != -1)
  {
    // Broadcast the robot's pose.
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates();
    map_x = cor_x;
    map_y = cor_y;
    map_theta = cor_theta;
    
    // BPP logic
    if (state == BPP)
      state = easyBPP();
    if (state == TURNL)
      {
        if (map_theta > Initial_theta - PI4Turn)
          SweepBot->turn_left(Regular_speed);
        else
        {
          state = BPP;
          Initial_theta = map_theta;
        }
      }
    if (state == TURNR)
      {
        if (map_theta < Initial_theta + PI4Turn)
          SweepBot->turn_right(Regular_speed);
        else
        {
          state = BPP;
          Initial_theta = map_theta;
        }
      }

  }
  

  
  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  return 0;
}

int easyBPP()
{
  // The coordinate of 2 cells ahead
  int font_x = map_x + 2*cos(map_theta);
  int font_y = map_y + 2*sin(map_theta);
  // The coordinate of 1 cell left
  int left_x = map_x + sin(map_theta);
  int left_y = map_y - cos(map_theta);
  // The coordinate of 1 cell right
  int right_x = map_x - sin(map_theta);
  int right_y = map_y + cos(map_theta);

  // Motion logic
  if (mat.Point(font_x,font_y)==1)
  {
    if (mat.Point(right_x,right_y)==1)
    {
      if (mat.Point(left_x,left_y)==1)
        SweepBot->stop();
      else
        return TURNL;
    }
    else if (mat.Point(left_x,left_y)==1)
    {
      if (mat.Point(right_x,right_y)==1)
        SweepBot->stop();
      else
        return TURNR;
    }
    else 
        return TURNR;
  }
  else
    SweepBot->forward(Regular_speed);

  return BPP;
}
