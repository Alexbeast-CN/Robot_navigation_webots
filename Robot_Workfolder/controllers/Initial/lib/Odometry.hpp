#pragma once
#include <iostream>
#include <vector>

#define WHEEL_RADIUS 0.0205
#define ROBOT_RADIUS 0.0355
#define PI 3.141592653589793116


using namespace webots;
class Odometry
{
private:
    float l_last_angle;
    float r_last_angle;
    float l_delta_angle;   
    float r_delta_angle; 
    float forward_contribution;
    float theta_contribution;
    float Current_left;
    float Current_right;
public:
    float x;             
    float y;            
    float th;

    // Odometry(/* args */);
    Odometry()
    {
        l_last_angle = 0.00;
        r_last_angle = 0.00;
        l_delta_angle = 0;   
        r_delta_angle = 0; 
        x = 0;             
        y = 0;            
        th = 0; 
    }
    inline float Cordinates(float a , float b)
    {  
        Current_left = a;
        Current_right = b;

        l_delta_angle = Current_left - l_last_angle;
        r_delta_angle = Current_right - r_last_angle;
  
        l_last_angle = Current_left;
        r_last_angle = Current_right;

        forward_contribution = ((WHEEL_RADIUS * l_delta_angle)/2) + ((WHEEL_RADIUS * r_delta_angle)/2);
        theta_contribution = (l_delta_angle-r_delta_angle)*(0.5*(WHEEL_RADIUS)/ROBOT_RADIUS);
  
        x  = x + (forward_contribution * cos(th));
        y  = y + (forward_contribution * sin(th));
        th = th - theta_contribution;

        if(th > (2*PI))
        {
        th = th - (2*PI);
        }
        return x;
    }
    ~Odometry();
};
