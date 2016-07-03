
#include "TankDrive.h"
#include "freeram.h"
#include "I2Cdev.h"
#include "mpu.h"

 //Create TankDrive (speedPinLeft, SpeedPinRight, forwardPinLeft, ReversePinLeft, forwardPinRight, revesePinRight)
   TankDrive MainDrive(10, 5, 9, 8, 7, 6);

void setup() {
   //Setup Communication Defaults
    Fastwire::setup(400, 0);    
    Serial.begin(9600);
    delay(100);

  //Setup default Trapzoid motion values, Speeds and Feeds
    MainDrive.SetMaxSpeed(180);
    MainDrive.SetMinSpeed(50);
    MainDrive.SetTimeAccel(1000);

  //Initialize the MPU
    MainDrive.InitializeMPU();
    Serial.print("Free mem: "); Serial.println(freeRam());
 		
	//drive Forward Command in milliseconds
	  MainDrive.driveForward(4000);

}

void loop() {
  //put your main code here, to run repeatedly:

}
