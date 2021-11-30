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
Map easymap;
Node *robot_node;
Supervisor *supervisor;
double Regular_speed = 20;

// Load the map
Matrix mat = easymap.easyMapS();
int map_x;
int map_y;
double map_theta;
float Initial_theta = 0.0;
float cor_x;
float cor_y;
float z;
int turn_count = 0;

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
  // Setup supervisor
  Supervisor *supervisor = (Supervisor *)robot;
  robot_node = supervisor->getFromDef("SWEEP");

  // do this once only
  if (robot_node == NULL) {
    std::cerr << "No DEF Sweep node found in the current world file" << std::endl;
    exit(1);
  }
  Field *trans_field = robot_node->getField("translation");
  Field *rotate_field = robot_node->getField("rotation");

  /************************************* Loop ********************************************/
  while (robot->step(TIME_STEP) != -1)
  {
    // Broadcast the robot's pose.
    // std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates();
    const double *trans_values = trans_field->getSFVec3f();
    const double *rotate_values = rotate_field->getSFRotation();

    //printf("The rotation values are:%.2f, %.2f, %.2f, %.2f\n",rotate_values[0],rotate_values[1],rotate_values[2],rotate_values[3]);

    cor_x = (trans_values[0] + 0.4)*10;
    cor_y = -(trans_values[2] - 0.4)*10;
    map_theta = rotate_values[3];
    z = rotate_values[2];

    map_x = round(cor_x + 1);
    map_y = round(cor_y + 1);
    int behind_x = round(map_x - sin(map_theta));
    int behind_y = round(map_y - cos(map_theta));

    cout << "x is: " << map_x << " y is: " << map_y << endl;
    printf( "theta is: %.3f\n", map_theta);

    // cout << "behind x is: " << behind_x << " behind y is: " << behind_y << endl;
    // Show the map with tarjectory
    if (mat.Point(behind_x,behind_y)==0)
      mat += easymap.markTrajectoryS(behind_x,behind_y);
    mat.Show();

    // BPP logic
    if (state == BPP)
      state = easyBPP();
    else if (state == TURNL)
      {
        if (map_theta > Initial_theta - PI4Turn + 0.05)
          SweepBot->turn_left(Regular_speed);
        else
        {
          turn_count++;
          state = BPP;
          Initial_theta = Initial_theta - PI4Turn;


        }
      }
    else if (state == TURNR)
      {
        if (map_theta < Initial_theta + PI4Turn - 0.05)
          SweepBot->turn_right(Regular_speed);
        else
        {
          turn_count++;
          state = BPP;
          Initial_theta = Initial_theta + PI4Turn;
        }
      }
    else if (state == Astar)
      SweepBot->stop();

  }
  

  
  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  delete supervisor;

  return 0;
}


/*********************************** Functions ***************************************/

int easyBPP()
{
  int E = 30;

  // The coordinate of 1 cells ahead
  int font_x = round(map_x + sin(map_theta));
  int font_y = round(map_y + cos(map_theta));
  // The coordinate of 1 cell left
  int left_x = round(map_x + cos(map_theta));
  int left_y = round(map_y - sin(map_theta));
  // The coordinate of 1 cell right
  int right_x = round(map_x - cos(map_theta));
  int right_y = round(map_y + sin(map_theta));

  // Motion logic
  if (mat.Point(font_x,font_y)>=1)
  {
    if (mat.Point(right_x,right_y)>=1)
    {
      if (mat.Point(left_x,left_y)>=1)
      {
        mat += easymap.markTrajectoryS(map_x,map_y);
        return Astar;
      }
      else
      {
        //cout<<"Wall turn left!" << endl;
        return TURNR;
      }
    }
    else if (mat.Point(left_x,left_y)>=1)
    {
      if (mat.Point(right_x,right_y)>=1)
      {
        mat += easymap.markTrajectoryS(map_x,map_y);
        return Astar;
      }
      else
      {
        //cout<<"Wall turn right!" << endl;
        return TURNL;
      }
    }
    else 
    {
      //cout<<"Wall in the front!" << endl;
      return TURNL;
    }
  }
  else
  {
    // Synchronize turnning state      
    if (!(turn_count/2))
    {
      if (z<0)
      {
        map_theta = -map_theta;
      }
    }
    else
    {
      if (z>0)
      {
        map_theta = -map_theta;
      }
    }


    float turn_velocity;
    float e_thata;

    // Let robot follow a straight line after unperfect turnning.
    if (Initial_theta  < 3.141 && Initial_theta > 3.139)
    {
      if (map_theta<0)
      {
        e_thata = Initial_theta + map_theta;
        turn_velocity = E*e_thata; 

        SweepBot->setSpeed(Regular_speed + turn_velocity, Regular_speed - turn_velocity);
      }
      else
      {
        e_thata = Initial_theta - map_theta;
        turn_velocity = E*e_thata; 
        SweepBot->setSpeed(Regular_speed + turn_velocity, Regular_speed - turn_velocity);
      }
      cout << "When thate is 3.14 turning Speed is: " << turn_velocity << endl;
    }
    else
    {
      e_thata = Initial_theta -map_theta;
      turn_velocity = E*e_thata;
      SweepBot->setSpeed(Regular_speed + turn_velocity, Regular_speed - turn_velocity);
      cout << "When theta is 0 turning Speed is: " << turn_velocity << endl;
    }
  }

  return BPP;
}
