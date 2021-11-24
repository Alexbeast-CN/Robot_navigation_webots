#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <iostream>
#include <tuple>
#include <math.h>
#include "SweepRobot.hpp"

SweepRobot *SweepBot;
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
    float x;             
    float y;            
    float th;

public:
    Odometry()
    {
        l_last_angle = 0.00;
        r_last_angle = 0.00;
        l_delta_angle = 0;   
        r_delta_angle = 0; 
        x = 0;             
        y = 0;            
        th = 0; 
        Current_left = 0;
        Current_right = 0;
    }
    inline std::tuple<float,float,float> Cordinates()
    {  
        Current_left = SweepBot->leftposition();
        Current_right = SweepBot->rightposition();
        l_delta_angle = Current_left - l_last_angle;
        r_delta_angle = Current_right - r_last_angle;
  
        l_last_angle = Current_left;
        r_last_angle = Current_right;

        forward_contribution = ((WHEEL_RADIUS * l_delta_angle)/2) + ((WHEEL_RADIUS * r_delta_angle)/2);
        theta_contribution = (r_delta_angle-l_delta_angle)*(0.5*(WHEEL_RADIUS/ROBOT_RADIUS));
  
        x  = x + (forward_contribution * 100 * cos(th));
        y  = y + (forward_contribution * 100 * sin(th));
        th = th - theta_contribution;

        if(th > (2*PI))
        {
        th = th - (2*PI);
        }

        printf("x: %.3f",x);
        printf("\ty: %.3f\n",y);
        printf("theta: %.3f\n",th);
        return std::make_tuple(x,y,th);
    }
};

#endif