# Function descriptions

## 1. Class: SweepRobot

### 1.1 Time function

**bool step()**

step one time unit

**bool step(int count)**

step count time units

**void delay_ms( float ms )** 

delay a few ms

### 1.2 Motion function

> The actual max speed of the robot is 6.28m/s. But the input speed for the functions below is between 0 and 100.

**void setSpeed(double leftSpeed, double rightSpeed)**

Set up motor steep for left and right motors.

**void forward(double speed)**

Set a forward speed for the robot.

**void rotate_left(double degree)**

Let the robot rotate left to some degree.

**void rotate_right(double degree)**

Let the robot rotate right to some degree.

**void turn_around_left(double speed)**

Let the robot turn left pi. The parameter it needs is turning speed.

The main reason to set this function is to let the robot turn around before hitting an obstacle.

**void turn_around_right(double speed)**

Let the robot turn right pi. The parameter it needs is turning speed.

The main reason to set this function is to let the robot turn around before hitting an obstacle.

**void stop()**

Stop the robot's motion.

