#include <stdio.h>     
#include <math.h>      
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h> 
#include <webots/position_sensor.h>

// simulation time step is 32ms
#define TIME_STEP 32  

// 3 IR ground sensors
#define NB_GROUND_SENS 3
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

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
void updateKinematics();
struct Kinematics_s pose;

// robot speeds
#define Max_speed 1.5

/*
WbDeviceTag gs[NB_GROUND_SENS]; 
float gs_value[NB_GROUND_SENS] = {0, 0, 0};
*/

// Motors
WbDeviceTag left_motor, right_motor;

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];

// Proximity Sensors
#define NB_PS 8
WbDeviceTag ps[NB_PS];
float ps_value[NB_PS] = {0, 0, 0, 0, 0, 0, 0, 0};

// Position Sensors (encoders)
WbDeviceTag left_position_sensor, right_position_sensor;

void setup( );
void loop( );
void delay_ms( float ms );

// movement functions
void moving_forwards();
void stop_moving();
void rotate_right(float time);
void rotate_left(float time);

int main(void) {

  // Initialise Webots - must be done!
  wb_robot_init();

  // Code called here happens before the 
  // simulator begins running properly.
  setup();

  // Run the simulator forever.
  while( wb_robot_step(TIME_STEP) != -1 ) {
      
    // Call robot controller loop
    loop();
  }


  return 0;
}


void setup() {

  // Initialise simulation devices.
  char name[20];
  int i;
  
  // Setup LEDs
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  
  // Setup Ground Sensors
  /*
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }
  */

  // Setup motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
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
  
  // Setup proximity sensors
  for (i = 0; i < 8; i++) {
    
    // get distance sensors 
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  // You can add your own initialisation code here:
}

void loop()
{
  moving_forwards();
  delay_ms(300);
  stop_moving();
  delay_ms(100);
  updateKinematics();

  delay_ms( 200 );
}

void delay_ms( float ms ) {
  float millis_now;
  float millis_future;
  
  millis_now = wb_robot_get_time() * 1000.0;
  millis_future = millis_now + ms;
  
  // Wait for the elapsed time to occur
  // Note, steps simulation, so blocks
  // any further updates the rest of the code.
  while( millis_now < millis_future ) {
    millis_now = wb_robot_get_time() * 1000.0;
    wb_robot_step( TIME_STEP );
  } 
  
  return;
}


// movement functions
void moving_forwards()
{
    wb_motor_set_velocity(left_motor, Max_speed);
    wb_motor_set_velocity(right_motor, Max_speed);
}

void stop_moving()
{
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void rotate_right(float time)
{
    wb_motor_set_velocity(left_motor, Max_speed);
    wb_motor_set_velocity(right_motor, -Max_speed);
    delay_ms(time);
}

void rotate_left(float time )
{
    wb_motor_set_velocity(left_motor, -Max_speed);
    wb_motor_set_velocity(right_motor, Max_speed);
    delay_ms(time);
}

void updateKinematics() 
{

  // Get current wheel angular positions
  float Current_left = wb_position_sensor_get_value(left_position_sensor);
  float Current_right = wb_position_sensor_get_value(right_position_sensor);
  printf("Current_Left: %.2f, Current_Right: %.2f\n", Current_left, Current_right);

  pose.l_delta_angle = Current_left - pose.l_last_angle;
  pose.r_delta_angle = Current_right - pose.r_last_angle;
  // printf("Delta_left: %.2f, Delta_right: %.2f, totol: %.2f\n", pose.l_delta_angle, pose.r_delta_angle, pose.l_delta_angle + pose.r_delta_angle );

  pose.l_last_angle = Current_left;
  pose.r_last_angle = Current_right;

  float forward_contribution = ((WHEEL_RADIUM * pose.l_delta_angle)/2) + ((WHEEL_RADIUM * pose.r_delta_angle)/2);
  float theta_contribution = (pose.l_delta_angle-pose.r_delta_angle)*(0.5*(WHEEL_RADIUM)/WhEEL_DISTANCE_HALF);
  // printf("for_contri: %.2f, theta_contri: %.2f\n", forward_contribution,theta_contribution);

  pose.x  = pose.x + (forward_contribution * cos(pose.th));
  pose.y  = pose.y + (forward_contribution * sin(pose.th));
  pose.th = pose.th - theta_contribution;

  if(pose.th > (2*M_PI))
  {
    pose.th = pose.th - (2*M_PI);
  }
  printf("X %.2lf，Y %.2lf，angle %.2lf\n", pose.x, pose.y, pose.th);
  
}