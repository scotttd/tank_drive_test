/*
	 Trapezoid.cpp - Library to  find trapezoid profile position
	 Created by Scott Davidson, June 28 2016.
	 Released into the public domain
*/

#include <Arduino.h>
#include "Trapezoid.h"

//Update function to use each time one of the inputs change
void Trapezoid::updateTrapezoidValues()
{
	_timeMaxSpeed = _runTime - _timeAccel;
	_addSpeed = ((double) _maxSpeed - (double) _minSpeed)/(double) _timeAccel;
}

Trapezoid::Trapezoid()
{
	_timeAccel = 1000;
	_minSpeed = 130;
	_maxSpeed = 230;
	_startTime = 0;
	_runTime = 0;
}

bool Trapezoid::IsValid()
{
	return true;
}
Trapezoid::Trapezoid(int timeAccel,int minSpeed,int maxSpeed,long startTime,long runTime)
{
	_timeAccel = timeAccel;
	_minSpeed = minSpeed;
	_maxSpeed = maxSpeed;
	_startTime = startTime;
	_runTime = runTime;
	
	//Update the calculated values in the class
	updateTrapezoidValues();
}


int Trapezoid::getSetPoint(long cTime)
{
    int cSpeed = 0;
	if (cTime < _timeAccel) {
        cSpeed = _minSpeed + (cTime * _addSpeed);
    }
    else if (cTime < _timeMaxSpeed) {
        cSpeed = _maxSpeed;
    }
    else {
        cSpeed = _maxSpeed - ((cTime - _timeMaxSpeed) * _addSpeed);
    }
	//Make sure the value is within the Min and Max speed
	cSpeed = constrain(cSpeed, _minSpeed, _maxSpeed);
	return cSpeed;
}

void Trapezoid::setRunTime(long runTime)
{
	_runTime = runTime;
	//Update the calculated values in the class
	updateTrapezoidValues();
}

void Trapezoid::setAcceleration(int timeAccel)
{
  _timeAccel = timeAccel;
  //Update the calculated values in the class
  updateTrapezoidValues();
}

void Trapezoid::setMaxSpeed(int maxSpeed)
{
  _maxSpeed = maxSpeed;
  //Update the calculated values in the class
  updateTrapezoidValues();
}

void Trapezoid::setMinSpeed(int minSpeed)
{
  _minSpeed = minSpeed;
  //Update the calculated values in the class
  updateTrapezoidValues();
}

void Trapezoid::setStartTime(long startTime)
{
	_startTime = startTime;
	//Update the calculated values in the class
	updateTrapezoidValues();
}


