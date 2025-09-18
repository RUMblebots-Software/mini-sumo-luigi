// UPRM RUMblebots Combact Robots Team
// Edimar Valentin Kery <edimar.valentin@upr.edu>
// Juan E. Quintana Gonzalez <juan.quintana5@upr.edu>
// OOP & Abstraction refactor: Yadriel Rivera Rodríguez <yadriel.rivera@upr.edu>
#include "Luigi.hpp"
SharpIR left_sensor(SharpIR::GP2Y0A21YK0F, *LEFT_SENSOR);
SharpIR right_sensor(SharpIR::GP2Y0A21YK0F, *RIGHT_SENSOR);
SharpIR ang_right_sensor(SharpIR::GP2Y0A21YK0F, *RIGHT_ANGLE_SENSOR);
SharpIR ang_left_sensor(SharpIR::GP2Y0A21YK0F, *LEFT_ANGLE_SENSOR);
SharpIR front_right_sensor(SharpIR::GP2Y0A21YK0F, *RIGHT_FRONT_SENSOR);
SharpIR front_left_sensor(SharpIR::GP2Y0A21YK0F, *LEFT_FRONT_SENSOR);
SharpIR back_sensor(SharpIR::GP2Y0A21YK0F, *BACK_SENSOR);

L3G gyro; // Create the gyro object


/**
 * TODO: eliminate magic numbers
 */

Luigi *LuigiObj = new Luigi(0,1,4,3,2,5,6,A7,A8,A2,A1,A11,A9,A6,A10,A0,A3);


void setup() {
  // put your setup code here, to run once:38400
  Serial.begin(9600);
  pinMode(*RIGHT_SENSOR, INPUT);
  pinMode(*RIGHT_ANGLE_SENSOR, INPUT);
  pinMode(*RIGHT_FRONT_SENSOR, INPUT);
  pinMode(*LEFT_FRONT_SENSOR, INPUT);
  pinMode(*LEFT_ANGLE_SENSOR, INPUT);
  pinMode(*LEFT_SENSOR, INPUT);
  pinMode(*BACK_SENSOR, INPUT);

  pinMode(*RIGHT_LINE_SENSOR, INPUT);
  pinMode(*LEFT_LINE_SENSOR, INPUT);
  pinMode(*BACK_LINE_SENSOR, INPUT);

  pinMode(*STBY, OUTPUT);
  pinMode(*PWMA, OUTPUT);
  pinMode(*PWMB, OUTPUT);
  pinMode(*AIN1, OUTPUT);
  pinMode(*AIN2, OUTPUT);
  pinMode(*BIN1, OUTPUT);
  pinMode(*BIN2, OUTPUT);

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


int speed = 255;
void loop() {
  int LeftLineReading = analogRead(*LEFT_LINE_SENSOR);
  int RightLineReading = analogRead(*RIGHT_LINE_SENSOR);
  Luigi *LuigiObj = new Luigi(0,1,4,3,2,5,6,A7,A8,A2,A1,A11,A9,A6,A10,A0,A3);
  while(true){
    
    Serial.println("FORWARD");
    LuigiObj->forward(speed) ;
    delay(2000);


    
  }

    if(left_sensor.getDistance() < 10 && (LeftLineReading < 300 && RightLineReading < 300)){
    LuigiObj->left(speed);
  }
    while (ang_left_sensor.getDistance() < 10 && (LeftLineReading < 300 && RightLineReading < 300)) {
       LuigiObj->leftForward(speed);
      if (LeftLineReading > 300 || RightLineReading > 300) {
         LuigiObj->right(speed,180);
        break;
      }
    }

  if (right_sensor.getDistance() < 10 && (LeftLineReading < 300 && RightLineReading < 300)){
     LuigiObj->right(speed);  
  }

  while (ang_right_sensor.getDistance() < 10 && (LeftLineReading < 300 && RightLineReading < 300)) {
       LuigiObj->rightForward(speed);
      if (LeftLineReading > 300 || RightLineReading > 300) {
         LuigiObj->left(speed,180);
        break;
      }
  }

  if (back_sensor.getDistance() < 10 && ((front_left_sensor.getDistance() + front_right_sensor.getDistance()) / 2) > back_sensor.getDistance() + 10 && (LeftLineReading < 300 && RightLineReading < 300)) {
     LuigiObj->left(speed ,180);
   }
   while ((front_left_sensor.getDistance() < 10 && front_right_sensor.getDistance() < 10) && (LeftLineReading < 300 && RightLineReading < 300))
   {
     LuigiObj->forward(speed);
      if (LeftLineReading > 300 || RightLineReading > 300) {
       LuigiObj->right(speed,180);
      break;
      }
   }
   if (back_sensor.getDistance() >= 10 && front_left_sensor.getDistance() >= 10 && front_right_sensor.getDistance() >= 10){
      LuigiObj->stopMotors();
    }  
}
