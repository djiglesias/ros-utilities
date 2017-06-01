/**************************************************************************//**
  ROS NODE: CAMERA SWIVEL HOUSING

  Sweeps an RGBD camera (Intel Realsense R200) through 130 degrees of motion
  while publishing the camera rotation to /tf through ROSSERIAL. Driven by
  a 32 step Kuman Stepper Motor with a gear ratio of 64 to provide 2048 steps
  per rotation. ULN2003APG motor driver chip.

  Internal interrupt attached to millis() to set pubishing rate to ROS Topic.
  Published a geometry_msgs::TransformedStamped message to /tf which informs
  the camera node of the orientation of the camera for SLAM.

  External interrupt attached to a switch on digital pin 2 for zeroing the
  stepper motor on start up.

  In the Stepper.h library the sequence for the four control wire config was
  updated for the Kuman Stepper Motor to allow bidirectional motion. The
  modifications are outlined below.

  Step C0 C1 C2 C3 --> Outdated
     1  1  0  1  0
     2  0  1  1  0
     3  0  1  0  1
     4  1  0  0  1

   Step C0 C1 C2 C3 --> Updated
      1  0  0  1  1
      2  1  0  0  1
      3  1  1  0  0
      4  0  1  1  0

  Source: https://forum.arduino.cc/index.php?topic=143276.0

  @author  Duncan Iglesias
  @company Arkbro Inc.
  @date    May 2017

  @link ...

*****************************************************************************/
#include <ros.h>
#include <ros/time.h>
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

#include "CameraPan.h"

#define RANGE  760
#define PIN    2
#define RATE   20
#define RPM    5

CameraPan *camera;

ros::NodeHandle  nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

unsigned long last_interrupt_time = millis();


/**************************************************************************//**
  Main Setup

  Initiates the ROS nodes and subscribes to the ROS topics over ROSSERIAL.

******************************************************************************/
void setup()
{
  // Launch ROS node and set parameters.
  nh.initNode();
  broadcaster.init(nh);
  while(!nh.connected()) {nh.spinOnce();}
  
  // Set pin modes for interrupt.
  camera = new CameraPan();
  pinMode(PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN), interrupt, RISING);

  // Initiate camera.
  camera->setSpeed(RPM);
  camera->initiate();

  // ROS Transform publisher.
  t.header.frame_id = "/camera_base";
  t.child_frame_id = "/camera_link";
  t.transform.translation.x = 0.0000;
  t.transform.translation.y = 0.0000;
  t.transform.translation.z = 0.0296;

  // Attach interrupt.
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

}


/**************************************************************************//**
  Internal Interrupt Handler
******************************************************************************/
SIGNAL(TIMER0_COMPA_vect)
{
  unsigned long currentMillis = millis();

  // Debounce interrupt signal to avoid false positives.
  if ( currentMillis - last_interrupt_time > (1000L / RATE) )
  {
    // Update rotation angles.
    t.transform.rotation = tf::createQuaternionFromRPY(camera->getAngle(), 0.0, 0.0);  
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);

    nh.spinOnce();

    // Update time.
    last_interrupt_time = currentMillis;
  }

}


/**************************************************************************//**
  Main Loop

  Continuous loop responsible for polling stepper position and publishing the
  data to the ROS master.

******************************************************************************/
void loop() {

  while (nh.connected()) {
    camera->setDirection(CameraPan::Direction::FORWARD);
    for (int i = 0; i < RANGE; i++)
      camera->step();
  
    delay(500);
  
    camera->setDirection(CameraPan::Direction::BACKWARD);
    for (int i = 0; i < RANGE; i++)
      camera->step();
  
    delay(500);

  }
}


/**************************************************************************//**
  Interrupt Handler

  Links an interrupt to the CameraPan interrupt handler.

******************************************************************************/
void interrupt() {
  camera->interruptHandler();
}


/*****************************************************************************/
