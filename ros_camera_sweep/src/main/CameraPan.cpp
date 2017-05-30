/**************************************************************************//**
* CAMERA PAN CLASS: SWEEPING CAMERA MOUNT (180 DEGREES)
* 
* This class controls a Kuman 32 step stepper motor with a 64:1 gear ratio to
* sweep an Intel Realsense R200 depth camera through approximately 160 degrees
* of motion. The angular rotation is updated with every step and can be polled
* but not set.
*
* Compatible with Arduino Uno/Nano boards. 
*
* Limit switch uses interrupt 0 (pin 2) with a 200ms timeout to debounce any
* noise from cheap swith components. The interrupt is triggered on a FALLING
* event with the limit switch wired as follows:
*		- Normally Closed (NC) --> VCC
*		- Normally Open (NO)   --> GND
*		- Closed (C)           --> Signal (Pin 2)
*
* @author  Duncan Iglesias
* @company Arkbro Inc.
* @date    May 2017
*
* @link    http://www. *** .com
*
*****************************************************************************/
#include "CameraPan.h"

#define STEP            1
#define STEP_COUNT      32
#define GEAR_RATIO      64
#define STEP_PER_REV    2048
#define STEP_LIMIT		  380
#define INTERRUPT_LIMIT 200
#define STEPPER_RPM		  5 

#define PIN_1			      8
#define PIN_2 			    7
#define PIN_3 			    6
#define PIN_4			      5

#define HOME 			      0
#define BACKTRACK 		  80


/**************************************************************************//**
* Default Constructor
*
******************************************************************************/
CameraPan::CameraPan(): stepper(STEP_PER_REV, PIN_1, PIN_2, PIN_3, PIN_4),
	limit(false), step_count(0), last_interrupt_time(0), calibrated(false), 
	direction(CameraPan::Direction::FORWARD)
{
	// Initiate stepper.
	stepper.setSpeed(STEPPER_RPM);
}


CameraPan::CameraPan(int steps, int p1, int p2, int p3, int p4): 
	stepper(steps, p1, p2, p3, p4), limit(false), step_count(0), 
	last_interrupt_time(0), calibrated(false)
{
	// Initiate stepper.
	stepper.setSpeed(STEPPER_RPM);
}


/**************************************************************************//**
* Initiate Camera
*
******************************************************************************/
void CameraPan::initiate()
{
	home();
  calibrated = true;
  step_count = -STEP_LIMIT;

}


/**************************************************************************//**
* Set Stepper Speed
* 
******************************************************************************/
void CameraPan::setSpeed(byte rpm)
{
  stepper.setSpeed(int(rpm));

}


/**************************************************************************//**
* Get Camera Angle
*
******************************************************************************/
float CameraPan::getAngle()
{  
	if (calibrated)
		return (float(step_count) * (360.0 / 2048.0)) * PI / 180; // Radians.
	else
		return NULL;

}


/**************************************************************************//**
* Interrupt Handler
*
* Detects falling edge changes on interrupt 0 (pin 2) to signal the full
* rotation in one direction. 
* 
******************************************************************************/
void CameraPan::interruptHandler()
{
  // Time since program start.
  unsigned long interrupt_time = millis();

  // Debounce interrupt signal to avoid false positives.
  if (interrupt_time - last_interrupt_time > INTERRUPT_LIMIT) 
  {    
    limit = !limit;    
    last_interrupt_time = interrupt_time;
  }

}


/**************************************************************************//**
* Set Direction
*
******************************************************************************/
void CameraPan::setDirection( Direction dir) {
  this->direction = dir;
}


/**************************************************************************//**
* Step in Direction
*
******************************************************************************/
void CameraPan::step( ) {
  if (direction == Direction::FORWARD) {
    stepper.step(STEP);
    step_count++;
  }
  else
  {
    stepper.step(-STEP);
    step_count--;
  }
}


/**************************************************************************//**
* Find Home
* 
******************************************************************************/
void CameraPan::home()
{
  // Backtrack a touch.
  stepper.step(BACKTRACK);
  delay(500);

	// Advance until interrupt.
	while (!limit)
		stepper.step(-STEP);	
	delay(500);

	// Advance off of limit switch.
	stepper.step(BACKTRACK);
	delay(500);

}

/*****************************************************************************/
