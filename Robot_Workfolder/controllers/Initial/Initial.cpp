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
#define PI4Turn 3.1
SweepRobot *SweepBot;
Map easymap;
double Regular_speed = 40;

// Load the map
Matrix mat = easymap.easyMapS();
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
#define Astar 4

/************************************* Main ********************************************/
int main(int argc, char **argv)
{
  // Display the map
  //mat.Show();

  // Initial state
  state = BPP;

  Odometry Odo;
  float cor_x;
  float cor_y;
  float cor_theta;
  float Initial_theta = 0.0;
  // float t;
  
  /*
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
*/
  //create the Robot instance.
  Robot *robot = new Robot();
  SweepBot = new SweepRobot(robot);
  // Set a speed for robot

  /************************************* Loop ********************************************/
  while (robot->step(TIME_STEP) != -1)
  {
    // Broadcast the robot's pose.
    std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates();
    map_x = round(cor_x + 1);
    map_y = round(cor_y + 1);
    map_theta = cor_theta;
    int behind_x = round(map_x - cos(map_theta));
    int behind_y = round(map_y - sin(map_theta));

    cout << "x is: " << map_x << " y is: " << map_y << endl;
    cout << "theta is: " << map_theta << endl;
    // controller crashed here
    if (mat.Point(behind_y,behind_x)==0)
      mat += easymap.markTrajectoryS(behind_y,behind_x);
    mat.Show();

    // BPP logic
    if (state == BPP)
      state = easyBPP();
    else if (state == TURNL)
      {
        if (map_theta > Initial_theta - PI4Turn)
          SweepBot->turn_left(Regular_speed);
        else
        {
          state = BPP;
          Initial_theta = map_theta;
        }
      }
    else if (state == TURNR)
      {
        if (map_theta < Initial_theta + PI4Turn)
          SweepBot->turn_right(Regular_speed);
        else
        {
          state = BPP;
          Initial_theta = map_theta;
        }
      }
    else if (state == Astar)
      SweepBot->stop();

  }
  

  
  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  return 0;
}

int easyBPP()
{
  // The coordinate of 1 cells ahead
  int font_x = round(map_x + cos(map_theta));
  int font_y = round(map_y + sin(map_theta));
  // The coordinate of 1 cell left
  int left_x = round(map_x + sin(map_theta));
  int left_y = round(map_y - cos(map_theta));
  // The coordinate of 1 cell right
  int right_x = round(map_x - sin(map_theta));
  int right_y = round(map_y + cos(map_theta));

  // Motion logic
  if (mat.Point(font_y,font_x)>=1)
  {
    if (mat.Point(right_y,right_x)>=1)
    {
      if (mat.Point(left_y,left_x)>=1)
        return Astar;
      else
      {
        //cout<<"Wall turn left!" << endl;
        return TURNL;
      }
    }
    else if (mat.Point(left_y,left_x)>=1)
    {
      if (mat.Point(right_y,right_x)>=1)
        return Astar;
      else
      {
        //cout<<"Wall turn right!" << endl;
        return TURNR;
      }
    }
    else 
    {
      //cout<<"Wall in the front!" << endl;
      return TURNR;
    }
  }
  else
    SweepBot->forward(Regular_speed);

  return BPP;
}
