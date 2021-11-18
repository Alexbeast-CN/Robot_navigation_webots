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
#include "lib/Robot.hpp"

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
    inline vector<float> Cordinates(float Current_left, float Current_right)
    {  
        l_delta_angle = Current_left - l_last_angle;
        r_delta_angle = Current_right - r_last_angle;
  
        l_last_angle = Current_left;
        r_last_angle = Current_right;

        float forward_contribution = ((WHEEL_RADIUS * l_delta_angle)/2) + ((WHEEL_RADIUS * r_delta_angle)/2);
        float theta_contribution = (l_delta_angle-r_delta_angle)*(0.5*(WHEEL_RADIUS)/ROBOT_RADIUS);
  
        pose.x  = pose.x + (forward_contribution * cos(pose.th));
        pose.y  = pose.y + (forward_contribution * sin(pose.th));
        pose.th = pose.th - theta_contribution;

        if(pose.th > (2*M_PI))
        {
        pose.th = pose.th - (2*M_PI);
        }
        return {pose.x, pose.y, pose.th};
    }
private:
    float Current_left =  
    float Current_right = 
}