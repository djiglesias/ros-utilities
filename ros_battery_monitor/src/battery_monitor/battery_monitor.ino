/**************************************************************************//**
 * ROS NODE: VOLTAGE MONITORING SENSOR
 * 
 * Monitors the health of the power supply and provides individual cell
 * voltage and overall power availability.
 *
 * @author  Duncan Iglesias
 * @company Arkbro Inc.
 * @date    February 2017
 *
 * @link https://www.instructables.com/id/1S-6S-Battery-Voltage-Monitor-ROS/
 *
 *****************************************************************************/
#include <ros.h>
#include <sensor_msgs/BatteryState.h>

#define K           0.00472199
#define CELLS       6
#define MAX_CELLS   12
#define CRITICAL    0.30

double cell_const[MAX_CELLS] = 
{
  1.0000, 2.1915, 2.6970, 4.1111,
  4.7333, 6.6000, 6.6000, 7.8293,
  8.4667, 9.2353, 11.0000, 11.0000
};

ros::NodeHandle nh;
sensor_msgs::BatteryState batt_state;

ros::Publisher batteryState("/crawler/battery/info", &batt_state);


/**************************************************************************//**
* Main Setup
*
* Initiates the ROS nodes and subscribes to the ROS topics over ROSSERIAL. 
*
******************************************************************************/
void setup() 
{
  // Launch ROS node and set parameters.
  nh.initNode();

  // Setup publishers/subscribers.
  nh.advertise(batteryState);

  // Populate battery parameters.
  batt_state.design_capacity          = 2200;  // mAh
  batt_state.power_supply_status      = 2;     // discharging
  batt_state.power_supply_health      = 0;     // unknown
  batt_state.power_supply_technology  = 3;     // LiPo
  batt_state.present                  = 1;     // battery present

  batt_state.location      = "Crawler";        // unit location
  batt_state.serial_number = "ABC_0001";       // unit serial number
  
  batt_state.cell_voltage = new float[CELLS];  // individual cell health
  
}


/**************************************************************************//**
* Main Function
*
* Initiates the ROS nodes and subscribes to the ROS topics to respond to 
* incoming motor communication requests. Instantiates the motor controller
* and publishes the current motor position.
* 
* @param argc   --> Number of input arguements. 
* @aram argv**  --> Array of input arguments.
*
******************************************************************************/
void loop() 
{
  // Battery status.
  double battVoltage = 0.0;
  double prevVoltage = 0.0;

  // Reset Power Supply Health.
  batt_state.power_supply_health = 0;
  
  // Populate battery state message.
  for (int i = 0; i < CELLS; i++)
  {
    // Read raw voltage from analog pin.
    double cellVoltage = analogRead(i) * K;
    
    // Scale reading to full voltage.
    cellVoltage *= cell_const[i];
    double tmp = cellVoltage;
    
    // Isolate current cell voltage.
    cellVoltage -= prevVoltage;
    battVoltage += cellVoltage;
    prevVoltage = tmp;

    // Set current cell voltage to message.
    batt_state.cell_voltage[i] = (float)cellVoltage;

    // Check if battery is attached.
    if (batt_state.cell_voltage[i] >= 2.0)
    {
      if (batt_state.cell_voltage[i] <= 3.2)
        batt_state.power_supply_health = 5; // Unspecified failure.
      batt_state.present = 1;
    }
    else
      batt_state.present = 0;
  }

  // Update battery health.
  if (batt_state.present)
  {
    batt_state.voltage = (float)battVoltage;
    float volt = batt_state.voltage;
    float low  = 3.0 * CELLS;
    float high = 4.2 * CELLS;
    batt_state.percentage = constrain((volt - low) / (high - low), 0.0, 1.0);    
  }
  else 
  {
    batt_state.voltage = 0.0;
    batt_state.percentage = 0.0;
  }
  
  // Update power supply health if not failed.
  if (batt_state.power_supply_health == 0 && batt_state.present)
  {
    if (batt_state.voltage > CELLS * 4.2)
      batt_state.power_supply_health = 4; // overvoltage
    else if (batt_state.voltage < CELLS * 3.0)
      batt_state.power_supply_health = 3; // dead
    else
      batt_state.power_supply_health = 1; // good 
  }

  // Publish data to ROSSERIAL.
  batteryState.publish( &batt_state );
  nh.spinOnce();
  delay(1000);
  
}

/*****************************************************************************/
