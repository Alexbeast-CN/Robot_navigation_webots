#pragma once

#include <iostream>
#include <vector>
// #include <tuple>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64 // time in [ms] of a simulation step
#define MAX_SPEED 6.28 
#define UNIT_SPEED (MAX_SPEED / 100.0)
#define UNIT_FORWARD (UNIT_SPEED*WHEEL_RADIUS*2)
#define PI 3.141592653589793116
#define HALF_PI 1.570796326794896558
#define WHEEL_RADIUS 0.0205
#define ROBOT_RADIUS 0.0355 
#define ROBOT_DIAMETER 0.071
#define CELL 0.1


using namespace webots;

class SweepRobot
{
public:
    /**
     * Constructor
     * param robot  
     * 
     * Takes robot and create instance of motors, position sensor, 
     * distance sensors and light sensors
     * 
     **/
    SweepRobot(Robot *robot)
    {
        this->robot = robot;
        leftMotor = robot->getMotor("left wheel motor");
        rightMotor = robot->getMotor("right wheel motor");
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftSensor = leftMotor->getPositionSensor();
        rightSensor = rightMotor->getPositionSensor();
        leftSensor->enable(TIME_STEP);
        rightSensor->enable(TIME_STEP);

        char psNames[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

        for (int i = 0; i < 8; i++)
        {
            ps[i] = robot->getDistanceSensor(psNames[i]);
            ps[i]->enable(TIME_STEP);
        }
    }


    inline float leftposition()
    {
        return  leftSensor->getValue();
    }
    
    inline float rightposition()
    {
        return  rightSensor->getValue();
    }
/*
    inline std::tuple<float,float,float> Cordinates(float a , float b)
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
        return std::make_tuple(x,y,th);
    }
*/
    // step one time unit
    inline bool step()
    {
        return robot->step(TIME_STEP) != -1;
    }

    // step count time units
    inline bool step(int count)
    {
        for (int i = 0; i < count; ++i)
            if (!step())
                return false;
        return true;
    }

    

    // checks if wall exist on the right of bot
    inline bool wallRight()
    {
        return readDistanceSensor(2) > PS_THRESHOLD;
    }

    // checks if wall exist on the left of bot
    inline bool wallLeft()
    {
        return readDistanceSensor(5) > PS_THRESHOLD;
    }

    // checks if wall exist on the ahead of bot
    inline bool wallAhead()
    {
        return readDistanceSensor(0) > PS_THRESHOLD || readDistanceSensor(7) > PS_THRESHOLD;
    }

    // if bot as open space in front left or right
    inline bool openSpace()
    {
        return !(wallAhead() || wallRight() || wallLeft());
    }

    // read distance sensor
    inline double readDistanceSensor(int i)
    {
        return ps[i]->getValue();
    }

    // delay a few ms


    // sets speed to 0 and steps
    inline void stop()
    {
        setSpeed(0,0);
        step();
    }

    void setSpeed(double leftSpeed, double rightSpeed);
    void forward(double speed);
    void rotate_left(double time, double degree);
    void rotate_right(double time, double degree);
    void turn_around_left(double speed);
    void turn_around_right(double speed);
    void delay_ms( float ms );


    // destructor
    ~SweepRobot()
    {
        if (robot != nullptr)
            delete robot;
    }

private:
    Robot *robot;
    Motor *leftMotor, *rightMotor;
    PositionSensor *leftSensor, *rightSensor;
    LightSensor *ls[8];
    DistanceSensor *ps[8];

    const double PS_THRESHOLD = 80;
    const double PS_NOISE = 65;
    const double speed = 20;
};

// function to set speed of motors
    void SweepRobot::setSpeed(double leftSpeed, double rightSpeed)
    {
        leftMotor->setVelocity(leftSpeed * UNIT_SPEED);
        rightMotor->setVelocity(rightSpeed * UNIT_SPEED);
    }

    // Move the robot forward with some speed
    void SweepRobot::forward(double speed)
    {
        setSpeed(speed,speed);
    }

    // Let the robot rotate left to some degree
    void SweepRobot::rotate_left(double time, double degree)
    {
        setSpeed(-speed,speed);
        float real_speed = speed*UNIT_SPEED;
        float l_time = PI*ROBOT_DIAMETER*100*degree/real_speed/360/UNIT_FORWARD;
        robot->step(l_time);
    }

    
    // Let the robot rotate right to some degree
    void SweepRobot::rotate_right(double speed, double degree)
    {
        setSpeed(speed,-speed);
        float real_speed = speed*UNIT_SPEED;
        float r_time = PI*ROBOT_DIAMETER*100*degree/real_speed/360/UNIT_FORWARD;
        robot->step(r_time);
    }

    // Let the robot turn left pi
    void SweepRobot::turn_around_left(double speed)
    {  
        double real_speed = speed*UNIT_FORWARD;
        double route = (PI*CELL)/2;
        double vout = (CELL/2+ROBOT_RADIUS)/(CELL/2)*speed;
        double vin = (CELL/2-ROBOT_RADIUS)/(CELL/2)*speed;
        float time = route/real_speed*1000;
        std::cout<<time<<std::endl;
        setSpeed(vin,vout);
        robot->step(time);
    }

    // Let the robot turn right pi
    void SweepRobot::turn_around_right(double speed)
    {
        double real_speed = speed*UNIT_FORWARD;
        double vout = (CELL/2+ROBOT_RADIUS)/(CELL/2)*speed;
        double vin = (CELL/2-ROBOT_RADIUS)/(CELL/2)*speed;
        float time = (PI*CELL)/real_speed*1000;
        std::cout<<time<<std::endl;
        setSpeed(vout,vin);
        robot->step(time);
    }

    // delay function
    void SweepRobot::delay_ms( float ms ) 
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

