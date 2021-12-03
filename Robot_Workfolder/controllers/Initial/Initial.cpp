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

// STL
#include <iomanip>

// Type Define
typedef std::pair<int,int> Coordinate;

// Environment variables
#define PI4Turn 3.14
SweepRobot *SweepBot;
Map easymap;
Node *robot_node;
Supervisor *supervisor;
double Regular_speed = 20;
// Astar variables
Astar Path;
std::map<Coordinate,Coordinate> Route;
std::map<Coordinate,Coordinate> INV_Route;//inverse the order of the route
std::priority_queue<std::pair<int, std::pair<int, int>>> Closet_Point;

// Load the map
Matrix mat = easymap.easyMapS();
Matrix mat2 = easymap.easyMapSS();//build a static mpa for A*
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
#define Move 5


/************************************* Main ********************************************/
int main(int argc, char **argv)
{
  // Display the map
  //mat.Show();

  // Initial state
  state = BPP;

  // Odometry Odo;
  // float t;

  // create a link to store the robot's position in sequence
  std::vector<Coordinate> CCP_Path{Coordinate(1,1)};

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

  /************************************* Loop ********************************************/
  
  while (robot->step(TIME_STEP) != -1)
  {
    // Broadcast the robot's pose.
    // std::tie(cor_x,cor_y,cor_theta) = Odo.Cordinates();
    const double *trans_values = trans_field->getSFVec3f();
    const double *rotate_values = rotate_field->getSFRotation();

    // Get the robot's position from supervisor
    cor_x = (trans_values[0] + 0.4)*10;
    cor_y = -(trans_values[2] - 0.4)*10;
    z = rotate_values[2];
    map_x = cor_x + 1.15;
    map_y = cor_y + 1.15;
    map_theta = rotate_values[3];

    static int previous_x = 0;
    static int previous_y = 0;
    static int count = 0;
    int it = 0;

    // Broadcast the robot's position
    cout << "x is: " << map_x << " y is: " << map_y << endl;
    printf( "theta is: %.3f\n", map_theta);

    int mark_x = 0;
    int mark_y = 0;
    // When the robot position is changed, update the map
    if (previous_x != map_x || previous_y != map_y)
    {
      count++;
      previous_x = map_x;
      previous_y = map_y;
      // store the robot's position in sequence
      CCP_Path.push_back(Coordinate(map_x,map_y));
      it = CCP_Path.size();
      if (it > 2)
      {
        mark_x = CCP_Path[it-2].first;
        mark_y = CCP_Path[it-2].second;
        // Show the map with tarjectory
        if (mat.Point(mark_x,mark_y) < 1)
        {
          mat += easymap.markTrajectoryS(mark_x,mark_y);
        }
      }
    }

    cout << "cor_x is: " << cor_x << " cor_y is: " << cor_y << endl;
    cout << "map_x is: " << map_x << " map_y is: " << map_y << endl;
    cout << "mark_x is: " << mark_x << " mark_y is: " << mark_y << endl;
    // Show the map
    mat.Show();

    // State machine
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
      int length;
      Coordinate Start_value(map_x,map_y);
      Coordinate End_value;
      Coordinate Point_behind;
      Coordinate Find_Point;

      // cout<<"找到的目标点为"<<Get_End_value.first<<""<<Get_End_value.second<<endl;

      cout<<"进来的点x和y:"<<map_x<<map_y<<endl;
      for (int i=1; i<10; i++)
      {
        for (int j=1; j<10; j++)
        {
        // cout<<"hi"<<endl;
          if(mat.Point(i,j) == 0)
          {
            Coordinate end(i,j);
            length = abs(i - Start_value.first) + abs(j - Start_value.second);
            Closet_Point.push(std::make_pair(-length,end));
          }
        }
      }

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

      End_value = Find_Point;
      cout<<"找到的目标点为"<<End_value.first<<""<<End_value.second<<endl;
      Route = Path.Findpath(Start_value,End_value,easymap.easyMapSS());

      Point_behind = End_value;
      while(Route.count(Point_behind)>0)
      {
        INV_Route[Route[Point_behind]] = Point_behind;
        Point_behind = Route[Point_behind];
      }

      while(!Closet_Point.empty())
      {
        Closet_Point.pop();
      }

      state = Move;
    }

    else if (state == Move)
    {
      Coordinate Present_Cor;
      Present_Cor.first = map_x;
      Present_Cor.second = map_y;
      cout<<"X"<<Present_Cor.first<<"Y"<<Present_Cor.second<<endl;
      cout<<"下一个点为"<<INV_Route[Present_Cor].first<<INV_Route[Present_Cor].second<<endl;
      // movement 
      if(INV_Route[Present_Cor] != std::make_pair(0,0))
      {
        if(INV_Route[Present_Cor].second-Present_Cor.second == -1)
        {
          if(map_theta<0.04 && map_theta>-0.03)
          {
            static int i = 1;
            // 掉头180度
            SweepBot->rotate_left(Regular_speed);
            SweepBot->delay_ms(3540);
            if(i == 1)
            {
              SweepBot->forward(Regular_speed);
              SweepBot->delay_ms(2000);
              i = 2;
            }
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
            SweepBot->delay_ms(3900);
            state = Move;
          }
        }
        else if(INV_Route[Present_Cor].first-Present_Cor.first == -1)
        {
          if(map_theta>-3.15 && map_theta<-3)
          {
            SweepBot->rotate_right(Regular_speed);
            SweepBot->delay_ms(1720);
            state = Move;
          }
          else if(map_theta<3.14 && map_theta>3.12)
          {
            SweepBot->rotate_right(Regular_speed);
            SweepBot->delay_ms(1720);
            state = Move;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(3900);
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
          else if(map_theta<-1.5 && map_theta>-1.6)
          {
            SweepBot->rotate_left(Regular_speed);
            SweepBot->delay_ms(1720);
            state = Move;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(1000);
            state = Move;
          }
        }
        else if(INV_Route[Present_Cor].first-Present_Cor.first == 1)
        {
          if(map_theta>-3.15 && map_theta<-3)
          {
            SweepBot->rotate_left(Regular_speed);
            SweepBot->delay_ms(1720);
            state = Move;
          }
          else if(map_theta<3.14 && map_theta>3.12)
          {
            SweepBot->rotate_left(Regular_speed);
            SweepBot->delay_ms(1720);
            state = Move;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(3900);
            state = Move;
          } 
        }
      }
      else
      {
        cout<<"到达目标点"<<endl;
        Route.clear();
        INV_Route.clear();
        // mat += easymap.markTrajectoryS(map_x,map_y);
        state = BPP;
      }
    }
  }

  
  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  delete supervisor;

  return 0;
}


/*********************************** Functions ***************************************/

// CCP motion logic
int easyBPP()
{
  int E = 30;

  // The coordinate of 1 cells ahead
  int front_x = round(cor_x + 0.9 + sin(map_theta));
  int front_y = round(cor_y + 0.9 + cos(map_theta));
  // The coordinate of 1 cell left
  int left_x = round(cor_x + 0.9 + cos(map_theta));
  int left_y = round(cor_y + 0.9 - sin(map_theta));
  // The coordinate of 1 cell right
  int right_x = round(cor_x + 0.9 - cos(map_theta));
  int right_y = round(cor_y + 0.9 + sin(map_theta));

  // Motion logic
  if (mat.Point(front_x,front_y)>=1)
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
    else if (mat.Point(map_x,map_y)>=1)
    {
      //cout<<"Step on trajectory" << endl;
      return Astar;
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

