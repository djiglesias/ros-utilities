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

  @link https://www.instructables.com/id/Sweeping-Camera-Mount-ROS/

*****************************************************************************/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

#include "CameraPan.h"

#define RANGE  760
#define PIN    2
#define RATE   20
#define RPM    1

#define LAG    30

CameraPan *camera;

ros::NodeHandle  nh;
sensor_msgs::JointState joint_state;
ros::Publisher joint_msg_pub("/joint_states", &joint_state);


unsigned long last_interrupt_time = millis();


/**************************************************************************//**
  Main Setup

  Initiates the ROS nodes and subscribes to the ROS topics over ROSSERIAL.

******************************************************************************/
void setup()
{
  // Launch ROS node and set parameters.
  nh.initNode();
  nh.advertise(joint_msg_pub);

  // Resize Joint State Message.
  joint_state.name_length     = 1;
  joint_state.position_length = 1;
  
  joint_state.name     = new char*[1];
  joint_state.position = new float[1];
  char topic_name[]    = "camera_mount_joint";  
  joint_state.name[0]  = topic_name;

  // Set pin modes for interrupt.
  camera = new CameraPan();
  camera->setSpeed(RPM);
  pinMode(PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN), interrupt, RISING); 

  // Attach interrupt.
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  // Delay until connected to ROS master.
  while(!nh.connected()) {nh.spinOnce();}
  camera->initiate();

  // Reset lag occurance.
  camera->setDirection(CameraPan::Direction::FORWARD);
  for (int i = 0; i < LAG; i++)
    camera->step(0);  
  delay(100);
  
  camera->setDirection(CameraPan::Direction::BACKWARD);
  for (int i = 0; i < LAG; i++)
    camera->step(0);  
  delay(100);
  
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
    // Update Joint State Parameters.
    joint_state.header.stamp = nh.now();
    joint_state.position[0]  = -camera->getAngle();// - 1.5708;

    // Publish Joint State.
    joint_msg_pub.publish( &joint_state );
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

  if (nh.connected()) {
    camera->setDirection(CameraPan::Direction::FORWARD);
    for (int i = 0; i < LAG; i++)
      camera->step(0);  
    for (int i = 0; i < (RANGE - LAG); i++)
      camera->step(1);
    delay(250);
  
    camera->setDirection(CameraPan::Direction::BACKWARD);
    for (int i = 0; i < LAG; i++)
      camera->step(0);  
    for (int i = 0; i < (RANGE - LAG); i++)
      camera->step(1);
    delay(250);

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
