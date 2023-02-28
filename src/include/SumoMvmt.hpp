#include "include/SumoMvmt.h"

void forward(int speed)
{
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void rightForward(int speed)
{
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, speed * 0.55);
  analogWrite(PWMB, speed);
}

void leftForward(int speed)
{
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed * 0.55);
}

// Sets motors to stop and shuts down the motor driver. Use this whenever the sumo shouldn't move.
void stopMotors()
{
  digitalWrite(STBY, LOW);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// Spins right at x speed and stops a y angle
void SumoMvmt::right_angled(int speed, float angle)
{
  float Current_z_angle = 0.0f;
  unsigned long PrevTime = millis();
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);

  while (Current_z_angle <= angle)
  {
    Serial.println("Spinning");
    this->getGyro().read();
    float DPS = (float)this->getGyro().g.z * 0.00875; // Degrees Per Second
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

// turns right at speed x until it stops detecting something to the right
void SumoMvmt::right(int speed)
{
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);

  stopMotors();
}

// Spins left at x speed and stops a y angle
void SumoMvmt::left_angled(int speed, float angle)
{
  float Current_z_angle = 0.0f;
  unsigned long PrevTime = millis();
  digitalWrite(STBY, HIGH);
   // Motor1 + Motor 2 = Speed

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);

  while (Current_z_angle <= angle)
  {
    Serial.println("Spinning");
    this->getGyro().read();
    float DPS = (float)this->getGyro().g.z * 0.00875; // Degrees Per Second

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

// turns left at speed x until it stops detecting something to the left
void SumoMvmt::left(int speed)
{
  unsigned long PrevTime = millis();
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);

  stopMotors();
}

// Sets motors to go back at x speed
void reverse(int speed)
{
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}
// const float GYRO-DPS-PER-LSB = 0.00875;

int random(int lower, int upper)
{
  int num = (rand() % (upper - lower + 1)) + lower;
  return num;
}

// Build a default constructor