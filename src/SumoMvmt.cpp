#include "SumoMvmt.hpp"


/** NOTE: these values are merely placeholders, the real values are given at the construction of the MiniSumo object */ 
//These are the pins for the TB6612FNG Motor Driver
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

uint8_t* RIGHT_SENSOR = new uint8_t(A6);
uint8_t* RIGHT_ANGLE_SENSOR = new uint8_t(A7);
uint8_t* RIGHT_FRONT_SENSOR = new uint8_t(A3);

uint8_t* LEFT_FRONT_SENSOR = new uint8_t(A2);
uint8_t* LEFT_ANGLE_SENSOR = new uint8_t(A0);
uint8_t* LEFT_SENSOR = new uint8_t(A1);

uint8_t* BACK_SENSOR = new uint8_t(A10);

//These are the pins for the Dual Micro Line Sensor ML2
// #define RIGHT_LINE_SENSOR A8
// #define LEFT_LINE_SENSOR A11
// #define BACK_LINE_SENSOR A9
uint8_t* RIGHT_LINE_SENSOR = new uint8_t(A8);
uint8_t* LEFT_LINE_SENSOR = new uint8_t(A11);
uint8_t* BACK_LINE_SENSOR = new uint8_t(A9);

//Sets both motors to go forward at x speed
void SumoMvmt::forward(int speed){

    digitalWrite(*STBY, HIGH);
        
    digitalWrite(*AIN1, LOW);
    digitalWrite(*AIN2, HIGH);
    
    digitalWrite(*BIN1, LOW);
    digitalWrite(*BIN2, HIGH);

    analogWrite(*PWMA, speed);
    analogWrite(*PWMB, speed);
}

//Sets motors to go back at x speed a=d b=i 
void SumoMvmt::reverse(int speed){
    digitalWrite(*STBY, HIGH);

    digitalWrite(*AIN1, HIGH);
    digitalWrite(*AIN2, LOW);
    
    digitalWrite(*BIN1, HIGH);
    digitalWrite(*BIN2, LOW);

    analogWrite(*PWMA, speed);
    analogWrite(*PWMB, speed);
}

//turns right at speed x utnitl it stops detecting something to the right
void SumoMvmt::right(int speed)  {
    digitalWrite(*STBY, HIGH);
  
    digitalWrite(*AIN1, HIGH);
    digitalWrite(*AIN2, LOW);
    
    digitalWrite(*BIN1, LOW);
    digitalWrite(*BIN2, HIGH);

    analogWrite(*PWMA, speed);
    analogWrite(*PWMB, speed);

    while(right_sensor.getDistance() < 10|| ang_right_sensor.getDistance() < 10){
    }
    stopMotors();
}

//Spins right at x speed and stops a y angle
void SumoMvmt::right(int speed, float angle){
    float Current_z_angle = 0.0f;
        unsigned long PrevTime = millis();

        digitalWrite(*STBY, HIGH);
    
        digitalWrite(*AIN1, HIGH);
        digitalWrite(*AIN2, LOW);
        
        digitalWrite(*BIN1, LOW);
        digitalWrite(*BIN2, HIGH);
        
        analogWrite(*PWMA, speed);
        analogWrite(*PWMB, speed);
        
        while(Current_z_angle <= angle){
            Serial.println("Spinning");
            gyro.read();

            //const float GYRO-DPS-PER-LSB = 0.00875;
            float DPS = (float)gyro.g.z * 0.00875; // Degrees Per Second
            
            unsigned long CurrentTime = millis();
            unsigned long DeltaTime = CurrentTime - PrevTime;
            PrevTime = CurrentTime;
            
            float Delta_z_angle = abs((DPS / 1000) * DeltaTime);
            
            Current_z_angle += Delta_z_angle;
            Serial.print("Z angle: ");
            Serial.println(Current_z_angle);
            Serial.print("Angle: ");
            Serial.println(angle); 
        }
        stopMotors();
}

//turns left at speed x until it stops detecing someting to the left
void SumoMvmt::left(int speed){
    digitalWrite(*STBY, HIGH);
  
    digitalWrite(*AIN1, LOW);
    digitalWrite(*AIN2, HIGH);
    
    digitalWrite(*BIN1, HIGH);
    digitalWrite(*BIN2, LOW);
    
    analogWrite(*PWMA, speed);
    analogWrite(*PWMB, speed);

    while(left_sensor.getDistance() < 10 || ang_left_sensor.getDistance() < 10){
    }
    stopMotors();
}

//Spins left at x speed and stops a y angle
void SumoMvmt::left(int speed, float angle){

    float Current_z_angle = 0.0f;
    unsigned long PrevTime = millis();

    digitalWrite(*STBY, HIGH);; // Motor1 + Motor 2 = Speed

    digitalWrite(*AIN1, LOW);
    digitalWrite(*AIN2, HIGH);
    
    digitalWrite(*BIN1, HIGH);
    digitalWrite(*BIN2, LOW);
    
    analogWrite(*PWMA, speed);
    analogWrite(*PWMB, speed);
    
    while(Current_z_angle <= angle){
        Serial.println("Spinning");
        gyro.read();

        //const float GYRO-DPS-PER-LSB = 0.00875;
        float DPS = (float)gyro.g.z * 0.00875; // Degrees Per Second
        
        unsigned long CurrentTime = millis();
        unsigned long DeltaTime = CurrentTime - PrevTime;
        PrevTime = CurrentTime;
        
        float Delta_z_angle = abs((DPS / 1000) * DeltaTime);
        
        Current_z_angle += Delta_z_angle;
        Serial.print("Z angle: ");
        Serial.println(Current_z_angle);
        Serial.print("Angle: ");
        Serial.println(angle);
    }
    stopMotors();
}

void SumoMvmt::rightForward(int speed){

    digitalWrite(*STBY, HIGH);
        
    digitalWrite(*AIN1, LOW);
    digitalWrite(*AIN2, HIGH);
    
    digitalWrite(*BIN1, LOW);
    digitalWrite(*BIN2, HIGH);

    analogWrite(*PWMA, speed * 0.55);
    analogWrite(*PWMB, speed * 1.20);
    while (ang_right_sensor.getDistance() < 10 && analogRead(*LEFT_LINE_SENSOR) < 300 && analogRead(*RIGHT_LINE_SENSOR) < 300) {
    }
    stopMotors();
}

void SumoMvmt::leftForward(int speed){
    digitalWrite(*STBY, HIGH);
  
    digitalWrite(*AIN1, LOW);
    digitalWrite(*AIN2, HIGH);
    
    digitalWrite(*BIN1, LOW);
    digitalWrite(*BIN2, HIGH);

    analogWrite(*PWMA, speed * 1.20);
    analogWrite(*PWMB, speed * 0.55);
    while (ang_left_sensor.getDistance() < 10 && analogRead(*LEFT_LINE_SENSOR) < 300 && analogRead(*RIGHT_LINE_SENSOR) < 300) {
    }
    stopMotors();
}

//Sets motors to stop and shuts down the motor driver. Use this whenever the sumo shouldn't move.
void SumoMvmt::stopMotors(){
    digitalWrite(*STBY, LOW);
        
    digitalWrite(*AIN1, LOW);
    digitalWrite(*AIN2, LOW);
    
    digitalWrite(*BIN1, LOW);
    digitalWrite(*BIN2, LOW);

    analogWrite(*PWMA, 0);
    analogWrite(*PWMB, 0);
}