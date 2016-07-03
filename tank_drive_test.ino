
#include "TankDrive.h"

 //Create TankDrive (speedPinLeft, SpeedPinRight, forwardPinLeft, ReversePinLeft, forwardPinRight, revesePinRight)
   TankDrive MainDrive(10, 5, 9, 8, 7, 6);

void setup() {
   //Setup Communication Defaults
    Serial.begin(9600);
    delay(100);
/*
   Serial.print("1");
	//Fastwire::setup(400, 0);
   Serial.print('here');
*/
  //Setup default Trapzoid motion values, Speeds and Feeds
    MainDrive.SetMaxSpeed(180);
    MainDrive.SetMinSpeed(50);
    MainDrive.SetTimeAccel(1000);

    //Trapezoid cTrapezoid = MainDrive.GetTrapezoid();
    //int a = cTrapezoid._maxSpeed;
    //Serial.println(a);
 		
	//drive Forward Command in milliseconds
	  MainDrive.driveForward(4000);

}

void loop() {
  //put your main code here, to run repeatedly:

}
