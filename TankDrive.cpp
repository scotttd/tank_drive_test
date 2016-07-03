/*
	 TankDrive.cpp - Library to create TankDrive
	 Created by Scott Davidson, June 28 2016.
	 Released into the public domain
*/

#include <Arduino.h>
#include "TankDrive.h"
#include "Trapezoid.h"
#include "mpu.h"

TankDrive::TankDrive(int speedPinLeft, int speedPinRight, int forwardPinRight, int reversePinRight, int forwardPinLeft, int reversePinLeft) :TrapPath()
{
	// connect motor controller pins to Arduino digital pins
	// motor Left
	_speedPinLeft = speedPinLeft;
	_forwardPinLeft = forwardPinLeft;
	_reversePinLeft = reversePinLeft;
	// motor two
	_speedPinRight = speedPinRight;
	_forwardPinRight = forwardPinRight;
	_reversePinRight = reversePinRight;

	// set all the motor control pins to outputs
	pinMode(speedPinLeft, OUTPUT);
    pinMode(speedPinRight, OUTPUT);
    pinMode(forwardPinLeft, OUTPUT);
    pinMode(reversePinLeft, OUTPUT);
    pinMode(forwardPinRight, OUTPUT);
    pinMode(reversePinRight, OUTPUT);

}

int TankDrive::InitializeMPU()
{
  int notStable = 1;
  int stableRead = 0;
  int oldAngle=180;
  long calibrationStart = millis();
  _mpuRet = mympu_open(200);
  Serial.print("MPU init: "); Serial.println(_mpuRet);
  Serial.println("Starting MPU stabilization check. This can take a minute.");
  Serial.println("The MPU requires 8 seconds of no movement to calibrate the Gyro with Accelerometer.");
  delay(8000); //Delay 2 seconds to settle the minitbot for MPU initialization
  while (notStable){ 
    _mpuRet = mympu_update();
    _currentAngle = mympu.ypr[0];
    if (_currentAngle == oldAngle) {
      stableRead=stableRead+1;
      }
    else {
      stableRead=0;
    }
    //Serial.println(_currentAngle);
    if (stableRead > 8000 and notStable){
      Serial.print("Stablised in ");Serial.print((millis()-calibrationStart)/1000);Serial.print(" seconds");
      Serial.print(" at ");Serial.print(_currentAngle);Serial.println(" degrees.");
      notStable=0;
    }
    oldAngle=_currentAngle;
   }
}


void TankDrive::fullStop ()
{
  // now turn off motors
  digitalWrite(_forwardPinLeft, LOW);
  digitalWrite(_reversePinLeft, LOW);
  digitalWrite(_forwardPinRight, LOW);
  digitalWrite(_reversePinRight, LOW);
}

void TankDrive::doDrive(int driveTime)
{
	  long startTime = millis();
	  long cTime = 0;
	  int cSpeed = 150;


    TrapPath.setRunTime(driveTime);
    TrapPath.setStartTime(startTime);

   while (millis() - startTime < driveTime) {
    _mpuRet = mympu_update();
    _currentAngle = mympu.ypr[0];
    //Serial.println(_currentAngle);
    //Serial.println("not Angle");
    cSpeed = TrapPath.getSetPoint(millis()-startTime);
    //Serial.println(cSpeed);
    digitalWrite(_speedPinLeft, cSpeed);
    digitalWrite(_speedPinRight, cSpeed);
   }
   fullStop();
 }

void TankDrive::driveForward(int driveTime)
{
  // Set motor directions to move forward
  digitalWrite(_forwardPinLeft, HIGH);
  digitalWrite(_reversePinLeft, LOW);
  digitalWrite(_forwardPinRight, HIGH);
  digitalWrite(_reversePinRight, LOW);
  
  doDrive (driveTime);
}

void TankDrive::driveReverse(int driveTime)
{
  // Set motor directions to move in reverse
  digitalWrite(_forwardPinLeft, LOW);
  digitalWrite(_reversePinLeft, HIGH);
  digitalWrite(_forwardPinRight, LOW);
  digitalWrite(_reversePinRight, HIGH);
  
  doDrive (driveTime);
}


void TankDrive::turnLeft(int turnAngle)
{
	 // Set motor directions to Turn Left
	  digitalWrite(_forwardPinLeft, LOW);
	  digitalWrite(_reversePinLeft, LOW);
	  digitalWrite(_forwardPinRight, HIGH);
	  digitalWrite(_reversePinRight, LOW);
}

void TankDrive::turnRight(int turnAngle)
{
	// Set motor directions to turn Right
	  digitalWrite(_forwardPinLeft, HIGH);
	  digitalWrite(_reversePinLeft, LOW);
	  digitalWrite(_forwardPinRight, LOW);
	  digitalWrite(_reversePinRight, LOW);
}
/*
int TankDrive::TimeAccel() const
{
	return m_trapezoid._timeAccel;
}
*/
void TankDrive::SetTimeAccel(int timeAccel)
{
  TrapPath.setAcceleration(timeAccel);
}

void TankDrive::SetMaxSpeed(int maxSpeed)
{
  TrapPath.setMaxSpeed(maxSpeed);
}

void TankDrive::SetMinSpeed(int minSpeed)
{
  TrapPath.setMinSpeed(minSpeed);
}

/*Trapezoid& TankDrive::Trapezoid()
{
	return _trapezoid;
}
*/
Trapezoid& TankDrive::GetTrapezoid()
{
	return TrapPath;
}

/*void Trapezoid::SetTrapezoid(const Trapezoid& trapezoid)
{
	if (trapezoid.IsValid())
		m_trapezoid = trapezoid;
}

*/
