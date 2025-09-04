#pragma once

#include <Arduino.h>
#include <SharpIR.h>
#include <Wire.h> // This library allows to communicate with I2C devices
#include <L3G.h> // This is a library interfaces with L3GD20H, L3GD20, and L3G4200D gyros on Pololu boards
#include <string.h>



/**
 * TODO: make the pins and sensors a part of luigi
 */
//These are the pins for the TB6612FNG Motor Driver

// #define STBY 0
// #define PWMA 1
// #define PWMB 4
// #define AIN1 3
// #define AIN2 2
// #define BIN1 5
// #define BIN2 6
int* STBY = new int(0);
int* PWMA = new int(1);
int* PWMB = new int(4); 
int* AIN1 = new int(3);
int* AIN2 = new int(2);
int* BIN1 = new int(5);
int* BIN2 = new int(6);

//These are the pins for the Sharp GP2Y0A21YK0F Analog Distance Sensor

//#define RIGHT_SENSOR A6
// #define RIGHT_ANGLE_SENSOR A7
//#define RIGHT_FRONT_SENSOR A3
//#define LEFT_FRONT_SENSOR A2
// #define LEFT_ANGLE_SENSOR A0
// #define LEFT_SENSOR A1
// #define BACK_SENSOR A10

auto* RIGHT_SENSOR = new uint8_t(A6);
auto* RIGHT_ANGLE_SENSOR = new uint8_t(A7);
auto* RIGHT_FRONT_SENSOR = new uint8_t(A3);

auto* LEFT_FRONT_SENSOR = new uint8_t(A2);
auto* LEFT_ANGLE_SENSOR = new uint8_t(A0);
auto* LEFT_SENSOR = new uint8_t(A1);

auto* BACK_SENSOR = new uint8_t(A10);

//These are the pins for the Dual Micro Line Sensor ML2
// #define RIGHT_LINE_SENSOR A8
// #define LEFT_LINE_SENSOR A11
// #define BACK_LINE_SENSOR A9
auto* RIGHT_LINE_SENSOR = new uint8_t(A8);
auto* LEFT_LINE_SENSOR = new uint8_t(A11);
auto* BACK_LINE_SENSOR = new uint8_t(A9);


//extern declares the variable without defining it, preventing multiple definitions across files

extern L3G gyro;

extern SharpIR left_sensor;
extern SharpIR right_sensor;
extern SharpIR ang_right_sensor;
extern SharpIR ang_left_sensor;
extern SharpIR front_right_sensor;
extern SharpIR front_left_sensor;
extern SharpIR back_sensor;


class SumoMvmt {
    
    public:


    //Implement all pure virtual functions from MiniSumo class
    void forward(int speed){};

    void reverse(int speed){};

    void right(int speed){};

    void right(int speed, float angle) {};

    void left(int speed){};

    void left(int speed, float angle) {};

    void rightForward(int speed){};

    void leftForward(int speed){};

    void stopMotors() {};

};