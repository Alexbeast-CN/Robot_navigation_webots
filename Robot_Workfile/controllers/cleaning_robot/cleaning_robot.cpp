// File:          cleaning_robot.cpp
#include <webots/Robot.hpp>
#include <iostream>
#include <iomanip>    
#include <math.h>      
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp> 
#include <webots/PositionSensor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// simulation time step is 32ms
#define TIME_STEP 32  

// epunk size
#define WHEEL_RADIUM        0.0205
#define WhEEL_DISTANCE_HALF 0.0292

// Struct Definition
struct Kinematics_s {

  // Left wheel
  float l_last_angle;  // angular position of wheel in last loop()
  float l_delta_angle; // calucated difference in wheel angle.

  // Right wheel
  float r_last_angle;  // angular position of wheel in last loop()
  float r_delta_angle; // calucated difference in wheel angle.

  float x;             // Global x position of robot
  float y;             // Global y position of robot
  float th;            // Global theta rotation of robot.

}pose;

// Odometry
// void updateKinematics();
// Map
#define MAP_XLIM 9
#define MAP_YLIM 9
float map[MAP_XLIM][MAP_YLIM];

// robot speeds
#define Max_speed 1.5

void setup( );
void loop( );
void delay_ms( float ms );

// movement functions
void moving_forwards();
void stop_moving();
void rotate_right(float time);
void rotate_left(float time);

Motor *leftMotor;
Motor *rightMotor;
Robot *robot;

int main(int argc, char **argv) 
{
  // create the Robot instance.
  Robot *robot = new Robot();

  setup();

  while (robot->step(TIME_STEP) != -1) 
  {
    // Call robot controller loop
    loop();
  }

  delete robot;
  return 0;
}

void setup() 
{
  // Setup motors
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("left wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);


  /*
  // get a handler to the position sensors and enable them.
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  //Obs_Sensor and Struct
  wb_robot_step(TIME_STEP);
  wb_position_sensor_get_value(left_position_sensor);
  wb_position_sensor_get_value(right_position_sensor);
  pose.l_last_angle = 0.00;
  pose.l_delta_angle = 0; 
  pose.r_last_angle = 0.00;  
  pose.r_delta_angle = 0; 
  pose.x = 0;             
  pose.y = 0;            
  pose.th = 0;
  */  
} 

void loop()
{
  moving_forwards();
  delay_ms(3280);
  stop_moving();
  delay_ms(100);
  // updateKinematics();

  delay_ms( 200 );
}

void delay_ms( float ms ) 
{
  float millis_now;
  float millis_future;
  
  millis_now = robot->getTime() * 1000.0;
  millis_future = millis_now + ms;
  while( millis_now < millis_future ) 
  {
    millis_now = robot->getTime() * 1000.0;
    robot->step( TIME_STEP );
  } 
  
  return;
}

// movement functions
void moving_forwards()
{
  leftMotor->setVelocity(Max_speed);
  rightMotor->setVelocity(Max_speed);
}

void stop_moving()
{
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
}

void rotate_right(float time)
{
  leftMotor->setVelocity(Max_speed);
  rightMotor->setVelocity(-Max_speed);
  delay_ms(time);
}

void rotate_left(float time )
{
  leftMotor->setVelocity(-Max_speed);
  rightMotor->setVelocity(Max_speed);
  delay_ms(time);
}