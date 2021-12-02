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
Matrix mat2 = easymap.easyMapSS();//build a static mpa for A*
int map_x;
int map_y;
int map_xx;
int map_yy;
double map_theta;
float Initial_theta = 0.0;
float cor_x;
float cor_y;
float z;
int turn_count = 0;

// Astar variables
std::pair<int,int>End_Detect(int a, int b);
Astar Path;

std::map<std::pair<int,int>,std::pair<int,int> >Route;
std::map<std::pair<int,int>,std::pair<int,int> >INV_Route;//inverse the order of the route

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
#define Move 5
#define GAGA 6 //做一个跳板state 看它是否又会被卡住不动

/************************************* Main ********************************************/
int main(int argc, char **argv)
{
  // Display the map
  //mat.Show();

  // Initial state
  state = BPP;

  Odometry Odo;
  // float t;

  //create the Robot instance.
  Robot *robot = new Robot();
  SweepBot = new SweepRobot(robot);

  // Setup supervisor
  Supervisor *supervisor = (Supervisor *)robot;
  robot_node = supervisor->getFromDef("SWEEP");

  // Detect if supervisor is on this webots system
  if (robot_node == NULL) {
    std::cerr << "No DEF Sweep node found in the current world file" << std::endl;
    exit(1);
  }

  // Get translation and rotation data from supervior node
  Field *trans_field = robot_node->getField("translation");
  Field *rotate_field = robot_node->getField("rotation");
  
  // 测试a*
  
  mat += easymap.markTrajectoryS(4,1);
  mat += easymap.markTrajectoryS(4,2);
  mat += easymap.markTrajectoryS(5,1);
  mat += easymap.markTrajectoryS(5,2);
  mat += easymap.markTrajectoryS(5,3);
  mat += easymap.markTrajectoryS(6,1);
  mat += easymap.markTrajectoryS(6,2);
  mat += easymap.markTrajectoryS(7,3);
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
    map_xx = round(cor_x + 0.7);
    map_y = round(cor_y + 1);
    map_yy = round(cor_y + 0.5);
    int behind_x = round(map_x - sin(map_theta));
    int behind_y = round(map_y - cos(map_theta));

    cout << "x is: " << map_xx << " y is: " << map_yy << endl;
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
    {
      cout<<"我进来了"<<endl;
      Route.clear();
      INV_Route.clear();
      std::pair<int,int>Get_Start_value(map_x,map_y);
      std::pair<int,int>Get_End_value;
      std::pair<int,int>Point_behind;
      Get_End_value = End_Detect(map_x, map_y);
      cout<<"找到的目标点为"<<Get_End_value.first<<""<<Get_End_value.second<<endl;
      Route = Path.Findpath(Get_Start_value,Get_End_value,easymap.easyMapSS());

      Point_behind = Get_End_value;
      while(Route.count(Point_behind)>0)
      {
        INV_Route[Route[Point_behind]] = Point_behind;
        Point_behind = Route[Point_behind];
      }
      state = Move;
    }

    else if (state == Move)
    {
      std::pair<int,int>Present_Cor;
      Present_Cor.first = map_xx;
      Present_Cor.second = map_yy;
      cout<<"X"<<Present_Cor.first<<"Y"<<Present_Cor.second<<endl;
      cout<<"下一个点为"<<INV_Route[Present_Cor].first<<INV_Route[Present_Cor].second<<endl;
      // movement 
      if(INV_Route[Present_Cor] != std::make_pair(0,0))
      {
        if(INV_Route[Present_Cor].second-Present_Cor.second == -1)
        {
          if(map_theta<0.04 && map_theta>-0.03)
          {
            // 掉头180度
            SweepBot->rotate_left(Regular_speed);
            SweepBot->delay_ms(3540);
            state = Move;
          }
          else if(map_theta<1.6 && map_theta>1.5)
          {
            SweepBot->rotate_left(Regular_speed);
            SweepBot->delay_ms(1730);
            state = Move;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            // SweepBot->delay_ms(3700);
            state = Move;
          }
        }
        else if(INV_Route[Present_Cor].first-Present_Cor.first == -1)
        {
          if(map_theta>1.6 || map_theta<1.5)
          {
            SweepBot->rotate_right(Regular_speed);
            SweepBot->delay_ms(1720);
            state = Move;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            // SweepBot->delay_ms(3700);
            state = Move;
          }
        }
        else if(INV_Route[Present_Cor].second-Present_Cor.second == 1)
        {
          if(map_theta<1.6 && map_theta>1.5)
          {
            SweepBot->rotate_right(Regular_speed);
            SweepBot->delay_ms(1720);
            state = Move;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            // SweepBot->delay_ms(3700);
            state = Move;
          }
        }
      }
      else
      {
        cout<<"到达目标点"<<endl;
        mat += easymap.markTrajectoryS(map_x,map_y);
        SweepBot->stop();
        SweepBot->delay_ms(1000);
        state = GAGA;
      }
    }

    else if(state == GAGA)
    {
      cout<<"进入GAGA"<<endl;
      state = Astar;
    }
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

std::pair<int,int>End_Detect(int a, int b)
{ 
  int length;
  std::pair<int, int>start(a,b);
  std::priority_queue<std::pair<int, std::pair<int, int>>> Closet_Point;
  while(!Closet_Point.empty())
  {
    Closet_Point.pop();
  }

  cout<<"进来的点x和y:"<<a<<b<<endl;
  for (int i=1; i<10; i++)
  {
    for (int j=1; j<10; j++)
    {
      // cout<<"hi"<<endl;
      if(mat.Point(i,j) == 0)
      {
        std::pair<int,int>end(i,j);
        length = abs(i - start.first) + abs(j - start.second);
        Closet_Point.push(std::make_pair(-length,end));
      }
    }
  }

  std::pair<int,int>Find_Point;
  if(!Closet_Point.empty())
  {
    Find_Point.first = Closet_Point.top().second.first;
    Find_Point.second = Closet_Point.top().second.second;
  }
  else
  {
    Find_Point.first = 1;
    Find_Point.second = 1;
  }
  
  return Find_Point;
}
