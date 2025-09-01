#pragma once
#include "MiniSumo.h" // Include the MiniSumo base class, which contains all basic MiniSumo methods
#include <Arduino.h>
#include <SharpIR.h>
#include <Wire.h> // This library allows to communicate with I2C devices
#include <L3G.h> // This is a library interfaces with L3GD20H, L3GD20, and L3G4200D gyros on Pololu boards
#include <string.h>

/**
 * NOTE: Decided to include pin and sensor definitions here to avoid the assumption that all 
 *      mini sumos will use the same motor drivers and sensors.
 */
//These are the pins for the TB6612FNG Motor Driver 
#define STBY 0
#define PWMA 1
#define PWMB 4
#define AIN1 3
#define AIN2 2
#define BIN1 5
#define BIN2 6

//These are the pins for the Sharp GP2Y0A21YK0F Analog Distance Sensor
#define RIGHT_SENSOR A6
#define RIGHT_ANGLE_SENSOR A7
#define RIGHT_FRONT_SENSOR A3
#define LEFT_FRONT_SENSOR A2
#define LEFT_ANGLE_SENSOR A0
#define LEFT_SENSOR A1
#define BACK_SENSOR A10

//These are the pins for the Dual Micro Line Sensor ML2
#define RIGHT_LINE_SENSOR A8
#define LEFT_LINE_SENSOR A11
#define BACK_LINE_SENSOR A9


//extern declares the variable without defining it, preventing multiple definitions across files

extern L3G gyro;

extern SharpIR left_sensor;
extern SharpIR right_sensor;
extern SharpIR ang_right_sensor;
extern SharpIR ang_left_sensor;
extern SharpIR front_right_sensor;
extern SharpIR front_left_sensor;
extern SharpIR back_sensor;


class Luigi : public MiniSumo {
    
    public:

    Luigi(){ }

    //Implement all pure virtual functions from MiniSumo class
    void forward(int speed);

    void reverse(int speed) override; 

    void right(int speed) override; 

    void left(int speed) override;

    void rightForward(int speed) override;

    void leftForward(int speed) override;

    void stopMotors() override;

    // Luigi unique methods
    void left(int speed, float angle); 
    void right(int speed, float angle);
};