// Teank drive testing
#include "TankDrive.h"
#include "freeram.h"
#include "I2Cdev.h"
#include "mpu.h"

 //Create TankDrive (speedPinLeft, SpeedPinRight, forwardPinLeft, ReversePinLeft, forwardPinRight, revesePinRight)
   TankDrive MainDrive(10, 5, 9, 8, 7, 6);

// Blinking led variables 
unsigned long previousToggleLed = 0;   // Last time the led was toggled
bool ledState                   = 0;   // Current state of Led
const int kBlinkLed             = 13;  // Pin of internal Led

void setup() {
   //Setup Communication Defaults
    Fastwire::setup(400, 0);    
    Serial.begin(9600);
    delay(100);

  //Setup default Trapzoid motion values, Speeds and Feeds
    MainDrive.SetMaxSpeed(180);
    MainDrive.SetMinSpeed(50);
    MainDrive.SetTimeAccel(1000);
    
  //Setup Motor PID values Kp:Ki:Kd
    MainDrive.MotorPID.SetTunings(50,15,15);
    MainDrive.MotorPID.SetOutputLimits(-255,255);

  //Initialize the MPU
    MainDrive.InitializeMPU();
    Serial.print("Free mem: "); Serial.println(freeRam());
 		
	//drive Forward Command in milliseconds
	  MainDrive.driveForward(4000);
          delay(4000);
          MainDrive.turnLeft(160);

}

void loop() {
  //put your main code here, to run repeatedly:

}

// Toggle led state
void toggleLed()
{  
  ledState = !ledState;
  digitalWrite(kBlinkLed, ledState?HIGH:LOW);
}  
