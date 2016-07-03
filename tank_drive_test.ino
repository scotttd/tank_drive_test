
#include "TankDrive.h"
#include "freeram.h"
#include "I2Cdev.h"
#include "mpu.h"

 //Create TankDrive (speedPinLeft, SpeedPinRight, forwardPinLeft, ReversePinLeft, forwardPinRight, revesePinRight)
   TankDrive MainDrive(10, 5, 9, 8, 7, 6);

   int ret;
   long calibrationStart;
   long calibrationTime;
   int oldAngle=180;
   int stableRead=0;
   int stable=1;

//MPU Status
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void setup() {
   //Setup Communication Defaults
    Fastwire::setup(400, 0);    
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
    
    ret = mympu_open(200);
    Serial.print("MPU init: "); Serial.println(ret);
    Serial.print("Free mem: "); Serial.println(freeRam());
    delay(2000); //Delay 2 seconds to settle the minitbot for MPU initialization

    calibrationStart = millis();
 		
	//drive Forward Command in milliseconds
	  //MainDrive.driveForward(4000);

}

void loop() {
  //put your main code here, to run repeatedly:
   if (!(c%10)) { // output only every 10 MPU/DMP reads
    ret = mympu_update();
    int currentAngle = mympu.ypr[0];
    if (currentAngle == oldAngle) {
      stableRead=stableRead+1;
      }
    else {
      stableRead=0;
    }
    //Serial.println(currentAngle);
    if (stableRead > 1000 and stable){
      Serial.print("Stablised in ");Serial.print((millis()-calibrationStart)/1000);Serial.println(" seconds.");
      Serial.print("Stabilised at ");Serial.print(currentAngle);Serial.println(" degrees.");
      stable=0;
    }
    oldAngle=currentAngle;
   }

}
