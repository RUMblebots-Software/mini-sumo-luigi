// UPRM RUMblebots Combact Robots Team
// Edimar Valentin Kery <edimar.valentin@upr.edu>
// Juan E. Quintana Gonzalez <juan.quintana5@upr.edu>
// Jean P. I. Sánchez Félix <jeanpiere.sanchez@upr.edu>

#include <include/SumoMvmt.h>

//Global object variables
SumoMvmt SumoData;
const int GENspeed = SumoData.getGENspeed();
const int LineDetection = SumoData.getLineDetection();

// Sharp GP2Y0A21YK0F Analog Distance Sensors (Constructors)
SharpIR left_sensor(SharpIR::GP2Y0A21YK0F, LEFT_SENSOR);
SharpIR right_sensor(SharpIR::GP2Y0A21YK0F, RIGHT_SENSOR);
SharpIR ang_right_sensor(SharpIR::GP2Y0A21YK0F, RIGHT_ANGLE_SENSOR);
SharpIR ang_left_sensor(SharpIR::GP2Y0A21YK0F, LEFT_ANGLE_SENSOR);
SharpIR front_right_sensor(SharpIR::GP2Y0A21YK0F, RIGHT_FRONT_SENSOR);
SharpIR front_left_sensor(SharpIR::GP2Y0A21YK0F, LEFT_FRONT_SENSOR);
SharpIR back_sensor(SharpIR::GP2Y0A21YK0F, BACK_SENSOR);

void SumoMvmt::setup()
{
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

  // Initialize the gyro if found. If not, setup never finishes.
  if (!this->getGyro().init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1)
      ;
  }
  this->getGyro().enableDefault();
}

void SumoMvmt::loop()
{
  // random movement
  while (left_sensor.getDistance() > 10 && right_sensor.getDistance() > 10 && back_sensor.getDistance() > 10 && front_left_sensor.getDistance() > 10 && front_right_sensor.getDistance() > 10 && ang_right_sensor.getDistance() > 10 && ang_left_sensor.getDistance() > 10)
  {
    forward(GENspeed);
    if (analogRead(LEFT_LINE_SENSOR) > LineDetection || analogRead(RIGHT_LINE_SENSOR) > LineDetection)
    {
      int r = random(1, 2);
      if (r == 1)
      {
        left_angled(GENspeed, random(130, 180));
      }
      else
      {
        right_angled(GENspeed, random(130, 180));
      }
      break;
    }
  }

  // movement of bot in rotations
  if (left_sensor.getDistance() < 10)
  {
    left(GENspeed);
  }

  if (right_sensor.getDistance() < 10)
  {
    right(GENspeed);
  }

  // back sensor rotation
  if (back_sensor.getDistance() < 10 && ((front_left_sensor.getDistance() + front_right_sensor.getDistance()) / 2) > back_sensor.getDistance())
  {
    left_angled(GENspeed, 180);
  }

  // general forward movement
  while (front_left_sensor.getDistance() < 10 && front_right_sensor.getDistance() < 10)
  {
    forward(GENspeed);
    if (analogRead(LEFT_LINE_SENSOR) > LineDetection && analogRead(RIGHT_LINE_SENSOR) > LineDetection)
    {
      right_angled(GENspeed, 180);
      break;
    }
    if (analogRead(LEFT_LINE_SENSOR) > LineDetection)
    {
      left_angled(GENspeed, 90);
      break;
    }
    if (analogRead(RIGHT_LINE_SENSOR) > LineDetection)
    {
      right_angled(GENspeed, 90);
      break;
    }
  }

  // angle right movement
  while (ang_right_sensor.getDistance() < 10)
  {
    rightForward(GENspeed);
    if (analogRead(LEFT_LINE_SENSOR) > LineDetection && analogRead(RIGHT_LINE_SENSOR) > LineDetection)
    {
      right_angled(GENspeed, 180);
      break;
    }
    if (analogRead(LEFT_LINE_SENSOR) > LineDetection)
    {
      right_angled(GENspeed, 90);
      break;
    }
    if (analogRead(RIGHT_LINE_SENSOR) > LineDetection)
    {
      left_angled(GENspeed, 90);
      break;
    }
  }

  // angle left movement
  while (ang_left_sensor.getDistance() < 10)
  {
    leftForward(GENspeed);
    if (analogRead(LEFT_LINE_SENSOR) > LineDetection && analogRead(RIGHT_LINE_SENSOR) > LineDetection)
    {
      right_angled(GENspeed, 180);
      break;
    }
    if (analogRead(LEFT_LINE_SENSOR) > LineDetection)
    {
      right_angled(GENspeed, 90);
      break;
    }
    if (analogRead(RIGHT_LINE_SENSOR) > LineDetection)
    {
      left_angled(GENspeed, 90);
      break;
    }
  }

  // no movement check
}