/**************************************************************************//**
* CAMERA PAN CLASS: SWEEPING CAMERA MOUNT (130 DEGREES)
* 
* @author  Duncan Iglesias
* @company Arkbro Inc.
* @date    May 2017
*
* @link ...
*
*****************************************************************************/
#ifndef CAMERA_PAN_H_
#define CAMERA_PAN_H_

#include <Arduino.h>
#include "Stepper.h"

class CameraPan
{

public:
	CameraPan();
	CameraPan(int number_of_steps, int pin1, int pin2, 
								   int pin3, int pin4);

  enum Direction { FORWARD = 0, BACKWARD = 1 };
	
	void  initiate();
	float getAngle();
  void  setSpeed(byte rpm);
  void  step();
	void  interruptHandler();
  void  setDirection(Direction dir);


private:
	Stepper stepper;

	void home();

	unsigned long last_interrupt_time;
	signed long step_count;
  Direction direction;
	volatile bool limit;
	volatile bool calibrated;

};
#endif
/*****************************************************************************/
