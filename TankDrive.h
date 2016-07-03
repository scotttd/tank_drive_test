/*
	 TankDrive.h - Library to create TankDrive
	 Created by Scott Davidson, June 28 2016.
	 Released into the public domain
*/
#ifndef TankDrive_h
#define TankDrive_h

#include "Trapezoid.h"

class TankDrive

{
	public:
		TankDrive(int speedPinLeft, int speedPinRight, int forwardPinRight, int reversePinRight, int forwardPinLeft, int reversePinLeft);
		void fullStop();
		void driveForward(int driveTime);
		void driveReverse(int driveTime);
		void turnLeft(int turnAngle);
		void turnRight(int turnAngle);

		int TimeAccel() const;
		void SetTimeAccel(int timeAccel);
   void SetMaxSpeed (int maxSpeed);
   void SetMinSpeed (int minSpeed);

		Trapezoid& GetTrapezoid();


	//private:
		void doDrive(int driveTime);
		Trapezoid TrapPath;
		int _speedPinLeft;
		int _speedPinRight;
		int _forwardPinRight;
		int _reversePinRight;
		int _forwardPinLeft;
		int _reversePinLeft;
};

#endif
