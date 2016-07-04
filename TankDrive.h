/*
	 TankDrive.h - Library to create TankDrive
	 Created by Scott Davidson, June 28 2016.
	 Released into the public domain
*/
#ifndef TankDrive_h
#define TankDrive_h

#include "Trapezoid.h"
#include "PID_sd.h"
class TankDrive

{
	public:
		TankDrive(int speedPinLeft, int speedPinRight, int forwardPinRight, int reversePinRight, int forwardPinLeft, int reversePinLeft);
		void fullStop();
		void driveForward(int driveTime);
		void driveReverse(int driveTime);
		void turnLeft(int turnAngle);
		void turnRight(int turnAngle);
    bool _usePID;

    Trapezoid& GetTrapezoid();
    int TimeAccel() const;
		void SetTimeAccel(int timeAccel);
    void SetMaxSpeed (int maxSpeed);
    void SetMinSpeed (int minSpeed);

   int InitializeMPU();
   float _simpleKp;
   float _simpleDeadband;

   PID& GetPID();


	//private:
		// routine to drive straight
		void doDrive(int driveTime);
		//Create a Trapezoid class to calculate accelerationa nd deceleration
		Trapezoid TrapPath;

    PID MotorPID;

    //Pins for motor control
		int _speedPinLeft;
		int _speedPinRight;
		int _forwardPinRight;
		int _reversePinRight;
		int _forwardPinLeft;
		int _reversePinLeft;


    //Values for finding driection
    int _mpuRet;
    int _currentAngle;
};

#endif
