#pragma once

#include <Arduino.h>
#include <SharpIR.h>
#include <Wire.h> // This library allows to communicate with I2C devices
#include <L3G.h>  // This is a library interfaces with L3GD20H, L3GD20, and L3G4200D gyros on Pololu boards
#include <string.h>

class SumoMvmt{
    private:
        L3G gyro; // Create the gyro object

        // global variables
        const int GENspeed = 255;
        const int LineDetection;

    public:
    // These are the pins for the TB6612FNG Motor Driver
    #define STBY 0
    #define PWMA 1
    #define PWMB 4
    #define AIN1 3
    #define AIN2 2
    #define BIN1 5
    #define BIN2 6

    // These are the pins for the Sharp GP2Y0A21YK0F Analog Distance Sensor
    #define RIGHT_SENSOR A6
    #define RIGHT_ANGLE_SENSOR A7
    #define RIGHT_FRONT_SENSOR A3
    #define LEFT_FRONT_SENSOR A2
    #define LEFT_ANGLE_SENSOR A0
    #define LEFT_SENSOR A1
    #define BACK_SENSOR A10

    // These are the pins for the Dual Micro Line Sensor ML2
    #define RIGHT_LINE_SENSOR A8
    #define LEFT_LINE_SENSOR A11
    #define BACK_LINE_SENSOR A9

        // Getters
        L3G getGyro() { return gyro; };
        int getGENspeed() { return GENspeed; };
        int getLineDetection() { return LineDetection; };


        // Instance methods
        void forward(int);
        void rightForward(int);
        void leftForward(int);
        void stopMotors();
        void right_angled(int, float);
        void right(int);
        void left_angled(int, float);
        void left(int);
        void reverse(int);
        void setup();
        void loop();
        int random(int, int);
};
