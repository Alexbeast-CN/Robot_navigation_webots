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
int Astar_Path(int a, int b);
int Astar_Home(int a, int b);
void Order_Path(std::pair<int,int>x,int num,std::vector<std::pair<int,int>>&vec);
Astar Path;
std::map<std::pair<int,int>,std::pair<int,int> >Route;



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
    {
      state = Astar_Path(map_x, map_y);
      SweepBot->stop();
    }
    else if (state == 10)
    {
      cout<<"到位了"<<endl;
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

int Astar_Path(int a, int b)
{ 
  int length;
  std::pair<int, int>start(a,b);
  std::priority_queue<std::pair<int, std::pair<int, int>>> Closet_Point;
  int number_route;
  
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

  //找到的新终点的坐标pair
  std::pair<int,int>Find_Point = Closet_Point.top().second;
  cout<<"找到的距离最近的点为"<<Find_Point.first<<""<<Find_Point.second<<endl;

  /*
  // 新的a*
  Route = Path.Findpath(start, Find_Point, mat);
  // 计算总步数用来返回值
  number_route = Path.Number_in_Path(Find_Point);
  cout<<"别吓我"<<number_route<<endl;
  std::vector<std::pair<int,int>>Route_Inorder{Find_Point}; //生成一个ector储存排序后的Astar路径
  Order_Path(Find_Point,number_route,Route_Inorder);
  cout<<"别搞我"<<Route_Inorder[8].first<<Route_Inorder[8].second<<endl;

  //小车运动
  for(int i=0; i<number_route;i++)
  {
    float Astar_x,Astar_y,Astar_theta;
    std::tie(Astar_x,Astar_y,Astar_theta) = Odo.Cordinates();
    // cout << "x is: " << Astar_x << " y is: " << Astar_y << endl;
    cout << "theta is: " << Astar_theta << endl;
    if(Route_Inorder[number_route-i-1].second - Route_Inorder[number_route-i].second == 1)
    {
      if(Astar_theta == 0)
      {
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900);
      }
      else
      {
        SweepBot->rotate_left(Regular_speed);
        SweepBot->delay_ms(1680);
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900);
      }
    }
    else if(Route_Inorder[number_route-i-1].first - Route_Inorder[number_route-i].first == 1)
    {
      if(Astar_theta == 0)
      {
        cout<<"向右转90度"<<endl;
        //向右转90度
        SweepBot->rotate_right(Regular_speed);
        SweepBot->delay_ms(770);
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900);  
      }
      else
      {
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900);
      }
    }
      cout<<"完成了一次了"<<endl;
  }
  SweepBot->stop();
  SweepBot->delay_ms(1000);
  // 找到点后清空优先队列
  while(!Closet_Point.empty())
  {
    Closet_Point.pop();
  }
  // 清空容器map
  Route.erase(Route.begin(), Route.end());
  
  // 清空容器vector
  Route_Inorder.clear();
  */
  state = 10;
  

  return state;
}

int Astar_Home(int a, int b)
{
  int number_route;
  std::pair<int,int>start(a,b);
  std::pair<int,int>home(1,1);
  // 新的a*
  Route = Path.Findpath(start, home, mat2);
  // 计算总步数用来返回值
  number_route = Path.Number_in_Path(home);
  cout<<"别吓我"<<number_route<<endl;
  std::vector<std::pair<int,int>>Route_Inorder{home}; //生成一个ector储存排序后的Astar路径
  Order_Path(home,number_route,Route_Inorder);
  cout<<"别搞我"<<Route_Inorder[3].first<<Route_Inorder[3].second<<endl;

  //小车回家运动
  for(int i=0; i<number_route;i++)
  {
    float Astar_x,Astar_y,Astar_theta;

    // cout << "x is: " << Astar_x << " y is: " << Astar_y << endl;
    cout << "theta is: " << Astar_theta << endl;
    if(Route_Inorder[number_route-i-1].second - Route_Inorder[number_route-i].second == -1)
    {
      if(Astar_theta<0.05 && Astar_theta>-0.05)
      {
        SweepBot->rotate_left(Regular_speed);
        SweepBot->delay_ms(3460);
        SweepBot->stop();
        SweepBot->delay_ms(500);
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900);
      }
      else
      {
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900);
      }
    }
    else if(Route_Inorder[number_route-i-1].first - Route_Inorder[number_route-i].first == -1)
    {
      static int i = 1;
      if(i==1)
      {
        cout<<"向右 hh 转90度"<<endl;
        SweepBot->rotate_right(Regular_speed);
        SweepBot->delay_ms(770);
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900); 
        i = 2; 
      }
      else
      {
        SweepBot->forward(Regular_speed);
        SweepBot->delay_ms(1900);
      }
    }
      cout<<"完成了一次了"<<endl;
  }
  SweepBot->stop();
  state = 100;
  return state;
}

void Order_Path(std::pair<int,int>x,int num,std::vector<std::pair<int,int>>&vec)
{
  std::pair<int,int>Point_behind = x;
  for(int i=1;i<=num;i++)
  {
    Point_behind = Route[Point_behind];
    vec.push_back(Point_behind);
  }
}