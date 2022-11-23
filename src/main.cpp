// UPRM RUMblebots Combact Robots Team
// Edimar Valentin Kery <edimar.valentin@upr.edu>
// Juan E. Quintana Gonzalez <juan.quintana5@upr.edu>

#include <Arduino.h>
#include <SharpIR.h>
#include <Wire.h> // This library allows to communicate with I2C devices
#include <L3G.h> // This is a library interfaces with L3GD20H, L3GD20, and L3G4200D gyros on Pololu boards
#include <string.h>
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

L3G gyro; // Create the gyro object

// Sharp GP2Y0A21YK0F Analog Distance Sensors

SharpIR left_sensor(SharpIR::GP2Y0A21YK0F, LEFT_SENSOR);
SharpIR right_sensor(SharpIR::GP2Y0A21YK0F, RIGHT_SENSOR);
SharpIR ang_right_sensor(SharpIR::GP2Y0A21YK0F, RIGHT_ANGLE_SENSOR);
SharpIR ang_left_sensor(SharpIR::GP2Y0A21YK0F, LEFT_ANGLE_SENSOR);
SharpIR front_right_sensor(SharpIR::GP2Y0A21YK0F, RIGHT_FRONT_SENSOR);
SharpIR front_left_sensor(SharpIR::GP2Y0A21YK0F, LEFT_FRONT_SENSOR);
SharpIR back_sensor(SharpIR::GP2Y0A21YK0F, BACK_SENSOR);



//Sets both motors to go forward at x speed
void forward(int speed){
  digitalWrite(STBY, HIGH);
  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
 
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

//Sets motors to stop and shuts down the motor driver. Use this whenever the sumo shouldn't move.
void stopMotors(){
  digitalWrite(STBY, LOW);
  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
 
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

}

//Spins right at x speed and stops a y angle
void right(int speed, float angle){
  float Current_z_angle = 0.0f;
  unsigned long PrevTime = millis();
      digitalWrite(STBY, HIGH);
  
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
     
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    
      analogWrite(PWMA, speed);
      analogWrite(PWMB, speed);
  
  while(Current_z_angle <= angle){
      Serial.println("Spinning");
      gyro.read();
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
//turns right at speed x utnitl it stops detecting something to the right
void right(int speed){
      digitalWrite(STBY, HIGH);
  
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
     
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    
      analogWrite(PWMA, speed);
      analogWrite(PWMB, speed);
  
  while(right_sensor.getDistance() < 10|| ang_right_sensor.getDistance() < 10){
  }
  stopMotors();
}
//Spins left at x speed and stops a y angle
void left(int speed, float angle){
  float Current_z_angle = 0.0f;
  unsigned long PrevTime = millis();
      digitalWrite(STBY, HIGH);; // Motor1 + Motor 2 = Speed
  
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
     
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    
      analogWrite(PWMA, speed);
      analogWrite(PWMB, speed);
  
  while(Current_z_angle <= angle){
      Serial.println("Spinning");
      gyro.read();
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
void left(int speed) {
  unsigned long PrevTime = millis();
      digitalWrite(STBY, HIGH);
  
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
     
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    
      analogWrite(PWMA, speed);
      analogWrite(PWMB, speed);
  
  while(left_sensor.getDistance() < 10 || ang_left_sensor.getDistance() < 10){
  }
  stopMotors();
}

//Sets motors to go back at x speed
void reverse(int speed){
  digitalWrite(STBY, HIGH);
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
 
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}
//const float GYRO-DPS-PER-LSB = 0.00875;

void setup() {
  // put your setup code here, to run once:38400
  Serial.begin(9600);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(RIGHT_ANGLE_SENSOR, INPUT);
  pinMode(RIGHT_FRONT_SENSOR, INPUT);
  pinMode(LEFT_FRONT_SENSOR, INPUT);
  pinMode(LEFT_ANGLE_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(BACK_SENSOR, INPUT);

  pinMode(RIGHT_LINE_SENSOR, INPUT);
  pinMode(LEFT_LINE_SENSOR, INPUT);
  pinMode(BACK_LINE_SENSOR, INPUT);

  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Start communicating with the SDA (data line) and SCL (clock line) 
  Wire.begin();

  //Initialze the gyro if found. If not, setup never finishes. 
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  
  gyro.enableDefault();
}

void loop() {
  // checks before moving 
  

  





  // movement of bot in rotations

  if(left_sensor.getDistance() < 10){
    left(255);
  }

  if(right_sensor.getDistance() < 10){
    right(255);
  }



  // back sensor rotation
  

  // if (back_sensor.getDistance() < 10 && ((front_left_sensor.getDistance() + front_right_sensor.getDistance()) / 2) > back_sensor.getDistance() + 10) {
  //   left(255,180);
  // }

  if (back_sensor.getDistance() < 10 && ((front_left_sensor.getDistance() + front_right_sensor.getDistance()) / 2) > back_sensor.getDistance()) {
    left(255,180);
  }


  // general forward movement

  while (front_left_sensor.getDistance() < 10 && front_right_sensor.getDistance() < 10){
    forward(255);
    if (analogRead(LEFT_LINE_SENSOR) > 500) {
      left(255,90);
      break;
    }
    if (analogRead(RIGHT_LINE_SENSOR) > 500) {
      right(255,90);
      break;
    }
    if (analogRead(LEFT_LINE_SENSOR) > 500 && analogRead(RIGHT_LINE_SENSOR) > 500) {
      right(255,180);
      break;
    }
  }
  

  // no movement check 
  if (back_sensor.getDistance() >= 10 && front_left_sensor.getDistance() >= 10 && front_right_sensor.getDistance() >= 10 && ang_right_sensor.getDistance() >= 10 && ang_left_sensor.getDistance() >= 10){
    stopMotors();
  } 




}
