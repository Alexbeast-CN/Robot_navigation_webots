#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <cmath>
#include <stack>
#include <queue>
#include <unordered_map>
#include <math.h>

// Webots Lib
#include <webots/Robot.hpp>
// Costumer Lib
#include "lib/SweepRobot.hpp"

// Environment variables
using namespace webots;

class Odometry
{
public:
    Odometry(SweepRobot *I_Love_You_Last_Time)
    {
        float l_last_angle = 0.00;
        float r_last_angle = 0.00;
        float l_delta_angle = 0;   
        float r_delta_angle = 0; 
        float x = 0;             
        float y = 0;            
        float th = 0; 
    }
    inline vector<float> Cordinates()
    {  
        float Current_left = leftposition();
        float Current_right = rightposition();

        l_delta_angle = Current_left - l_last_angle;
        r_delta_angle = Current_right - r_last_angle;
  
        l_last_angle = Current_left;
        r_last_angle = Current_right;

        float forward_contribution = ((WHEEL_RADIUS * l_delta_angle)/2) + ((WHEEL_RADIUS * r_delta_angle)/2);
        float theta_contribution = (l_delta_angle-r_delta_angle)*(0.5*(WHEEL_RADIUS)/ROBOT_RADIUS);
  
        x  = x + (forward_contribution * cos(th));
        y  = y + (forward_contribution * sin(th));
        th = th - theta_contribution;

        if(th > (2*M_PI))
        {
        th = th - (2*M_PI);
        }
        return {x, y, th};
    }
private:
    float l_last_angle;
    float r_last_angle;
    float l_delta_angle;   
    float r_delta_angle; 
    float x;             
    float y;            
    float th;
    float Current_left;
    float Current_right;
}