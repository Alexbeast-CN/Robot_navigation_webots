#pragma once

#include <iostream>
#include <vector>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Supervisor.hpp>


#define TIME_STEP 64 // time in [ms] of a simulation step
#define MAX_SPEED 6.28 
#define UNIT_SPEED (MAX_SPEED / 100.0)
#define UNIT_FORWARD (UNIT_SPEED*WHEEL_RADIUS)
#define PI 3.141592653589793116
#define HALF_PI 1.570796326794896558
#define WHEEL_RADIUS 0.0205
#define ROBOT_RADIUS 0.0282
#define ROBOT_DIAMETER 0.0564

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

    // sets speed to 0 and steps
    inline void stop()
    {
        setSpeed(0,0);
        step();
    }

    void setSpeed(double leftSpeed, double rightSpeed);
    void forward(double speed);
    void rotate_left(double time);
    void rotate_right(double time);
    void turn_left(double speed);
    void turn_right(double speed);
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
    void SweepRobot::rotate_left(double time)
    {
        setSpeed(-speed,speed);
    }

    
    // Let the robot rotate right to some degree
    void SweepRobot::rotate_right(double speed)
    {
        setSpeed(speed,-speed);
    }

    // Let the robot turn left pi
    void SweepRobot::turn_left(double speed)
    {  
        double vout = (CELL/2+ROBOT_RADIUS)/(CELL/2)*speed;
        double vin = (CELL/2-ROBOT_RADIUS)/(CELL/2)*speed;
        setSpeed(vin,vout);
    }

    // Let the robot turn right pi
    void SweepRobot::turn_right(double speed)
    {
        double vout = (CELL/2+ROBOT_RADIUS)/(CELL/2)*speed;
        double vin = (CELL/2-ROBOT_RADIUS)/(CELL/2)*speed;
        setSpeed(vout,vin);
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

