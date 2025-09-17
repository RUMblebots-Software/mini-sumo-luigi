#pragma once

#include <Arduino.h>
#include <SharpIR.h>
#include <Wire.h> // This library allows to communicate with I2C devices
#include <L3G.h> // This is a library interfaces with L3GD20H, L3GD20, and L3G4200D gyros on Pololu boards
#include <string.h>




//These are the pins for the TB6612FNG Motor Driver

extern int* STBY;
extern int* PWMA;
extern int* PWMB;
extern int* AIN1;
extern int* AIN2;
extern int* BIN1;
extern int* BIN2;

extern uint8_t* RIGHT_SENSOR;
extern uint8_t* RIGHT_ANGLE_SENSOR;
extern uint8_t* RIGHT_FRONT_SENSOR;
extern uint8_t* LEFT_FRONT_SENSOR;
extern uint8_t* LEFT_ANGLE_SENSOR;
extern uint8_t* LEFT_SENSOR;
extern uint8_t* BACK_SENSOR;

extern uint8_t* RIGHT_LINE_SENSOR;
extern uint8_t* LEFT_LINE_SENSOR;
extern uint8_t* BACK_LINE_SENSOR;

// Sensor objects
extern L3G gyro;
extern SharpIR left_sensor;
extern SharpIR right_sensor;
extern SharpIR ang_right_sensor;
extern SharpIR ang_left_sensor;
extern SharpIR front_right_sensor;
extern SharpIR front_left_sensor;
extern SharpIR back_sensor;



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


    void forward(int speed);

    void reverse(int speed);

    void right(int speed);

    void right(int speed, float angle);

    void left(int speed);

    void left(int speed, float angle);

    void rightForward(int speed);

    void leftForward(int speed)  ;

    void stopMotors();

};