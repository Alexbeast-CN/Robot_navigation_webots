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
double map_theta;
float Initial_theta = 0.0;
float cor_x;
float cor_y;
float z;
int turn_count = 0;

// Astar variables
void Astar_Path(int a, int b,std::vector<std::pair<int,int>>&vec);
Astar Path;



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
#define FaceTo0 5

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

  // do this once only
  if (robot_node == NULL) {
    std::cerr << "No DEF Sweep node found in the current world file" << std::endl;
    exit(1);
  }
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
  int h = 1;

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
    {
      std::vector<std::pair<int,int>>Get_Route_Inorder;
      cout<<"都没你的天"<<endl;
      Astar_Path(map_x, map_y,Get_Route_Inorder);
      // 返回的路径坐标容器的大小
      int Route_size = Get_Route_Inorder.size()-1;
      for(int i=0; i<=Route_size; i++)
      {
        cout<<Get_Route_Inorder[i].first<<Get_Route_Inorder[i].second<<endl;
      }
      //cout<<"大小为"<<Route_size<<endl;
      int nn = 0;
      int mm = 0;
      // E-punk movemnet function
      for(int i=0; i<Route_size;i++)
      { 
        cout << "theta is: " <<map_theta<< endl;
        if(Get_Route_Inorder[Route_size-i-1].second - Get_Route_Inorder[Route_size-i].second == -1)
        {   
          if(map_theta < 0.04 && map_theta > 0.03 && nn == 0)
          {
            //掉头
            SweepBot->rotate_left(Regular_speed);
            SweepBot->delay_ms(3540);
            SweepBot->stop();
            SweepBot->delay_ms(500);
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(1900);
            nn = 1;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(3900);
          }
        }
        else if(Get_Route_Inorder[Route_size-i-1].first - Get_Route_Inorder[Route_size-i].first == -1)
        {
          if(map_theta < 0.04 && map_theta > 0.03 && nn == 1)
          {
            cout<<"向右转90度"<<endl;
            //向右转90度
            SweepBot->rotate_right(Regular_speed);
            SweepBot->delay_ms(1720);
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(3700);  
            nn = 0;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(3700);
          }
        }
        else if(Get_Route_Inorder[Route_size-i-1].second - Get_Route_Inorder[Route_size-i].second == 1)
        {
          if(map_theta > 0.04 && mm == 0)
          {
            SweepBot->rotate_right(Regular_speed);
            SweepBot->delay_ms(1720);
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(3700);
            mm = 1;
          }
          else
          {
            SweepBot->forward(Regular_speed);
            SweepBot->delay_ms(3700);
          }

        }
        cout<<"完成了一次了"<<endl;
      }
      Get_Route_Inorder.clear();
      SweepBot->stop();
      SweepBot->delay_ms(500);
      if(h == 1)
      {
        state = FaceTo0;
      }
      if(h == 2)
      {
        mat += easymap.markTrajectoryS(5,4);
        state = FaceTo0;
      }
      h++;
    }

    else if (state == FaceTo0)
    {
      mat += easymap.markTrajectoryS(map_x,map_y);
      if (mat.Point(map_x,map_y-1)==0)
      {
        cout<<"小车面向0转向！"<<endl;
        SweepBot->rotate_left(Regular_speed);
        SweepBot->delay_ms(1720);
        mat += easymap.markTrajectoryS(7,3);
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(4000);
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(4000);
        SweepBot->rotate_right(Regular_speed);
        SweepBot->delay_ms(1720);
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(4000); 
        SweepBot->stop();
        SweepBot->delay_ms(1000);
      }
      cout<<"state5 finished"<<endl;
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

void Astar_Path(int a, int b,std::vector<std::pair<int,int>>&vec)
{ 
  std::map<std::pair<int,int>,std::pair<int,int> >Route;
  int length;
  std::pair<int, int>start(a,b);
  std::priority_queue<std::pair<int, std::pair<int, int>>> Closet_Point;
  int number_route = 0;
  while(!Closet_Point.empty())
  {
    Closet_Point.pop();
  }
  // 清空容器map
  Route.erase(Route.begin(), Route.end());

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
  cout<<"找到的目标点为"<<Find_Point.first<<""<<Find_Point.second<<endl;
  // 新的a*
  Route = Path.Findpath(start, Find_Point, easymap.easyMapSS());
  // 计算总步数
  std::pair<int,int>point_behind(Find_Point);
  while(point_behind != start)
  {
    point_behind = Route[point_behind];
    number_route++;
  }
  cout<<"总部数"<<number_route<<endl;

  vec.push_back(Find_Point);
  // cout<<"别搞我"<<Route_Inorder[8].first<<Route_Inorder[8].second<<endl;
  std::pair<int,int>Point_behind(Find_Point);
  for(int i=1;i<=number_route;i++)
  {
    Point_behind = Route[Point_behind];
    vec.push_back(Point_behind);
  }
}
