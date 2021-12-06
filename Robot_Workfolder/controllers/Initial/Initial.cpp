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
#include <iostream>
#include <fstream>

// Type Define
typedef std::pair<int,int> Coordinate;

// State define
int state;
#define BPP   1
#define TURNL 2
#define TURNR 3
#define As 4
#define Move 5
#define END 6 
#define TURNLPI 7
#define TURNRPI 8
#define FACE0   9

// Environment variables
SweepRobot *SweepBot;
Map easymap;
Node *robot_node;
Supervisor *supervisor;
double Regular_speed = 20;
#define TURNPI 3.14
#define TURNPI2 1.57

// Astar variables
Astar Path;
std::map<Coordinate,Coordinate> Route;
std::priority_queue<std::pair<int, std::pair<int, int>>> Closet_Point;
Coordinate End_value;//inverse the order of the route

// Load the map
Matrix mat = easymap.easyMapS();
Matrix mat2 = easymap.easyMapS();//为astar路径显示地图
int map_x;
int map_y;
double map_theta;
float Initial_theta = 0.0;
float cor_x;
float cor_y;
float z;
float y;
int turn_count = 0;

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using std::cout;
using std::endl;

// Function prototypes:
int easyBPP(Matrix &mat);
int AstarMove();
void Balance();
int Astar_path();
int Face0 ();
int easyBSA(Matrix &matx);

/************************************* Main ********************************************/
int main(int argc, char **argv)
{

  // Display the map
  //mat.Show();

  // Initial state
  state = BPP;

  // Odometry Odo;

  // Initial time
  float t = 0.0;
  

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
    y = rotate_values[1];
    map_x = cor_x + 1.18;
    map_y = cor_y + 1.18;
    map_theta = rotate_values[3];
    cout << "map_theta read from map is: " << map_theta << endl;

    cout << "y is: " << y << endl;
    if (y > -0.01)
    {
      map_theta = -map_theta;
      cout << "map_theta after reverse is: " << map_theta << endl;    
    }

    static int previous_x = 0;
    static int previous_y = 0;
    static int count = 0;
    int it = 0;

    // Broadcast the robot's position
    cout << "map x is: " << map_x << " map y is: " << map_y << endl;
    // printf( "theta is: %.3f\n", map_theta);

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
        // Mark tarjectory on the map
        if (mat.Point(mark_x,mark_y) < 1)
        {
          mat += easymap.markTrajectoryS(mark_x,mark_y);
        }
      }
    }


    if(Initial_theta < -3.13)
      Initial_theta = 3.14;

    // printf("cor_x is %.3f, cor_y is %.3f\n", cor_x, cor_y);
    // cout << "map_x is: " << map_x << " map_y is: " << map_y << endl;
    // cout << "mark_x is: " << mark_x << " mark_y is: " << mark_y << endl;

    // State machine
    if (state == BPP)
    {
      state = easyBPP(mat);
    }
    else if (state == TURNL)
    {
      // Show the map
      mat.Show();

      // For debug
      cout << "Initial theta is: " << Initial_theta << endl;
      printf( "theta is: %.3f\n", map_theta);

      if (map_theta > Initial_theta - TURNPI2 + 0.05)
      {
        cout << "---------------------- Turn Left" << endl;
        SweepBot->turn_left(Regular_speed);
      }
      else
      {
        turn_count++;
        Initial_theta = Initial_theta - TURNPI2;
        if (mat.Point(map_x,map_y) < 1)
          state = BPP;
        else
          state = Move;
      }
    }
    else if (state == TURNR)
    {
      // Show the map
      mat.Show();

      if (Initial_theta + TURNPI2 > TURNPI + 0.05)
      {
        Initial_theta = -Initial_theta;
        map_theta = -abs(map_theta);
      }
      else if (Initial_theta > -0.1 && Initial_theta < 0.1)
      {
        Initial_theta = 0;
        map_theta = abs(map_theta);
      }

      cout << "Initial theta is: " << Initial_theta << endl;
      printf( "theta is: %.3f\n", map_theta);

      if (map_theta < Initial_theta + TURNPI2 - 0.05)
      {
        cout << "---------------------- Turn Right" << endl;
        SweepBot->turn_right(Regular_speed);
      }
      else
      {
        turn_count++;
        Initial_theta = Initial_theta + TURNPI2;
        if (mat.Point(map_x,map_y) < 1)
          state = BPP;
        else
          state = Move;
      }
    }
    else if (state == TURNLPI)
    {
      // Show the map
      mat.Show();

      // For debug
      cout << "Initial theta is: " << Initial_theta << endl;
      printf( "theta is: %.3f\n", map_theta);

      if (map_theta > Initial_theta - TURNPI + 0.05)
      {
        cout << "---------------------- Turn Left" << endl;
        SweepBot->turn_left(Regular_speed);
      }
      else
      {
        turn_count++;
        Initial_theta = Initial_theta - TURNPI;
        if (mat.Point(map_x,map_y) < 1)
          state = BPP;
        else
          state = Move;
      }
    }
    else if (state == TURNRPI)
    {
      // Show the ma
      mat.Show();

      if (Initial_theta + TURNPI2 > TURNPI + 0.05)
      {
        Initial_theta = -Initial_theta;
        map_theta = -abs(map_theta);
      }
      else if (Initial_theta > -0.1 && Initial_theta < 0.1)
      {
        Initial_theta = 0;
        map_theta = abs(map_theta);
      }

      cout << "Initial theta is: " << Initial_theta << endl;
      printf( "theta is: %.3f\n", map_theta);

      if (map_theta < Initial_theta + TURNPI - 0.05)
      {
        cout << "---------------------- Turn Right" << endl;
        SweepBot->turn_right(Regular_speed);
      }
      else
      {
        turn_count++;
        Initial_theta = Initial_theta + TURNPI;
        if (mat.Point(map_x,map_y) < 1)
          state = BPP;
        else
          state = Move;
      }
    }
    else if (state == As)
    {
      state = Astar_path();
    }
    else if (state == Move)
    {
      state = easyBSA(mat2); 
    }
    else if (state == FACE0)
    {
      state = Face0();
    }
    else if(state == END)
    {
      SweepBot->stop();
      break;
    }
  }

  // Read experment time
  t = robot->getTime();
  cout << "The robot end at:" << t << endl;
  
  // Export the route to a file
  ofstream file;
  file.open("/home/tim/Webots_lab/Robot_navigation_webots/Results/BCP_rounte_easyMap.csv");
	for (std::vector<Coordinate>::iterator ite = CCP_Path.begin(); ite != CCP_Path.end(); ite++)
  {
		file << ite->first << ", " << ite->second << endl;
	}
	file.close();


  // Enter exit cleanup code here.
  delete robot;
  delete SweepBot;
  delete supervisor;
  return 0;
}


/*********************************** Functions ***************************************/

// CCP motion logic
int easyBPP(Matrix &matx)
{
  cout<<"------------------------ state BPP"<<endl;
  // Show the map
  matx.Show();

  if (Initial_theta > 0.05 && Initial_theta < TURNPI-0.05)
    map_theta = abs(map_theta);

  // The coordinate of 1 cells ahead
  int front_x = round(cor_x + 1 + sin(map_theta));
  int front_y = round(cor_y + 1 + cos(map_theta));

  int front_xx = round(cor_x + 1 + 0.5*sin(map_theta));
  int front_yy = round(cor_y + 1 + 0.5*cos(map_theta));
  // The coordinate of 1 cell left
  int left_x = round(cor_x + 1 + cos(map_theta));
  int left_y = round(cor_y + 1 - sin(map_theta));
  // The coordinate of 1 cell right
  int right_x = round(cor_x + 1 - cos(map_theta));
  int right_y = round(cor_y + 1 + sin(map_theta));

  printf("front_x: %d, front_y: %d\n", front_x, front_y);

  // Motion logic

  if (matx.Point(map_x,map_y)>0)
  {
    //cout<<"Step on trajectory" << endl;
    return As;
  }

  if (matx.Point(front_x,front_y)>=1)
  {
    if (matx.Point(right_x,right_y)>=1)
    {
      if (matx.Point(left_x,left_y)>=1)
      {
        if (matx.Point(front_xx,front_yy)>1)
        {          
          return As;
        }
        else if (matx.Point(front_xx,front_yy)==1)
        {
          SweepBot->rotate_right(Regular_speed);
          SweepBot->delay_ms(3480);
          SweepBot->forward(Regular_speed);
          SweepBot->delay_ms(2000);
          Initial_theta = Initial_theta + TURNPI;
          return BPP;
        }
      }
      else
      {
        cout<<"Wall turn right!" << endl;
        return TURNRPI;
      }
    }
    else if (matx.Point(left_x,left_y)>=1)
    {
      if (matx.Point(right_x,right_y)>=1)
      {
        if (matx.Point(front_xx,front_yy)>=1)
        {          
          return As;
        } 
      }
      else
      {
        cout<<"Wall turn left!" << endl;
        return TURNLPI;
      }
    }

    else 
    {
      cout<<"Wall in the front!" << endl;
      return TURNLPI;
    }
  }
  else
    Balance();

  cout << "map_theta been used for motion calculation is: " << map_theta << endl;  

  return BPP;
}

void Balance()
{
  int E = 10;
  float turn_velocity;
  float e_thata;
  cout << "Initial_theta: " << Initial_theta << endl;

  // Let robot follow a straight line after unperfect turnning.
  if (Initial_theta > TURNPI - 0.05 && Initial_theta < TURNPI + 0.05)
  {
    if (map_theta<0)
    {
      e_thata = Initial_theta + map_theta;
      turn_velocity = E*e_thata; 

      SweepBot->setSpeed(Regular_speed - turn_velocity, Regular_speed + turn_velocity);
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
    e_thata = Initial_theta - map_theta;
    turn_velocity = E*e_thata;
    SweepBot->setSpeed(Regular_speed + turn_velocity, Regular_speed - turn_velocity);
    cout << "When theta is 0 turning Speed is: " << turn_velocity << endl;
  }
}

int Astar_path ()
{
  cout<<"---------------------------- state A*"<<endl;
      int length;
      Coordinate Start_value(map_x,map_y);
      Coordinate Point_behind;
      Coordinate Find_Point;
      Route.clear();
      // INV_Route.clear();

      while(!Closet_Point.empty())
      {
        Closet_Point.pop();
      }


      mat2 = easymap.easyMapS();//重新构造地图；
      
      cout<<"x begins at: "<<map_x << ", y begins at: "<<map_y<<endl;
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
        return END;
      }

      End_value = Find_Point;
      cout<<"The target point is:("<<End_value.first<<", "<<End_value.second<<")"<<endl;
      Route = Path.Findpath(Start_value,End_value,mat2);

      mat2 = easymap.easyMapS();

      // build a map for Astar
      for(int i=1; i<10; i++)
      {
        for(int j=1; j<10; j++)
        {
          mat2 += easymap.markTrajectoryS(i,j);
        }
      }

      Point_behind = End_value;
      for(int i=1; i<=20; i++)
      {
        if(Route.count(Point_behind)>0)
        {
          mat2 += easymap.markTrajectoryB(Point_behind.first,Point_behind.second); 
          Point_behind = Route[Point_behind];
          cout<<"point_behine"<<Point_behind.first<<Point_behind.second;
        }
        else
          break;
      }


      mat2 += easymap.markTrajectoryB(Start_value.first,Start_value.second);
      mat2.Show();
      return Move;
}

int Face0 ()
{
  // The coordinate of 1 cells ahead
  int front_x = round(cor_x + 1 + sin(map_theta));
  int front_y = round(cor_y + 1 + cos(map_theta));
  // The coordinate of 1 cell left
  int left_x = round(cor_x + 1 + cos(map_theta));
  int left_y = round(cor_y + 1 - sin(map_theta));
  // The coordinate of 1 cell right
  int right_x = round(cor_x + 1 - cos(map_theta));
  int right_y = round(cor_y + 1 + sin(map_theta));

  if (mat.Point(front_x,front_y) == 0)
    return BPP;
  else if (mat.Point(left_x,left_y) == 0)
  {
    SweepBot->rotate_right(Regular_speed);
    SweepBot->delay_ms(1720);
    Initial_theta = Initial_theta + TURNPI2;
    cout << "Initial_theta: " << Initial_theta << endl;
    return BPP;
  }
  else if (mat.Point(right_x,right_y) == 0)
  {
    SweepBot->rotate_left(Regular_speed);
    SweepBot->delay_ms(1720);
    Initial_theta = Initial_theta - TURNPI2;
    cout << "Initial_theta: " << Initial_theta << endl;
    return BPP;
  }
  else
  {
    return As;
  }
  
  
}

int easyBSA(Matrix &matx)
{
  cout<<"------------------------ state BSA"<<endl;
  // Show the map
  matx.Show();

  if (Initial_theta > 0.05 && Initial_theta < TURNPI-0.05)
    map_theta = abs(map_theta);

  // The coordinate of 1 cells ahead
  int front_x = round(cor_x + 1 + sin(map_theta));
  int front_y = round(cor_y + 1 + cos(map_theta));

  int front_xx = round(cor_x + 1 + 0.5*sin(map_theta));
  int front_yy = round(cor_y + 1 + 0.5*cos(map_theta));
  // The coordinate of 1 cell left
  int left_x = round(cor_x + 1 + cos(map_theta));
  int left_y = round(cor_y + 1 - sin(map_theta));
  // The coordinate of 1 cell right
  int right_x = round(cor_x + 1 - cos(map_theta));
  int right_y = round(cor_y + 1 + sin(map_theta));

  printf("front_x: %d, front_y: %d\n", front_x, front_y);

  // Motion logic

  if (matx.Point(map_x,map_y)>0)
  {
    //cout<<"Step on trajectory" << endl;
    return As;
  }

  if (matx.Point(front_x,front_y)>=1)
  {
    if (matx.Point(right_x,right_y)>=1)
    {
      if (matx.Point(left_x,left_y)>=1)
      {
        if (matx.Point(front_xx,front_yy)>1)
        {          
          return FACE0;
        }
        else if (matx.Point(front_xx,front_yy)==1)
        {
          SweepBot->rotate_right(Regular_speed);
          SweepBot->delay_ms(3480);
          SweepBot->forward(Regular_speed);
          SweepBot->delay_ms(2000);
          Initial_theta = Initial_theta + TURNPI;
          return BPP;
        }
      }
      else
      {
        cout<<"Wall turn right!" << endl;
        return TURNR;
      }
    }
    else if (matx.Point(left_x,left_y)>=1)
    {
      if (matx.Point(right_x,right_y)>=1)
      {
        if (matx.Point(front_xx,front_yy)>=1)
        {          
          return FACE0;
        } 
      }
      else
      {
        cout<<"Wall turn left!" << endl;
        return TURNL;
      }
    }

    else 
    {
      cout<<"Wall in the front!" << endl;
      return TURNL;
    }
  }
  else
    Balance();

  cout << "map_theta been used for motion calculation is: " << map_theta << endl;  

  return Move;
}