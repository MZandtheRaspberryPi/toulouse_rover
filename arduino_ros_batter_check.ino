#include <ros.h>
#include <sensor_msgs/BatteryState.h>

float K = 5.0/1023.0; // we'll read with analog, so need to trasnform 0-1023 range into 0-5v range, as we're reading voltage.
#define CELLS       2

int battPin = A0;

ros::NodeHandle nh;
sensor_msgs::BatteryState batt_state;
ros::Publisher batteryState("battery_state", &batt_state);

void setup()
{
  // Initialize the ROS node.
  nh.initNode();
  nh.advertise(batteryState);

  // Populate battery parameters.
  batt_state.design_capacity = 2200;      // mAh
  batt_state.power_supply_status = 2;     // discharging
  batt_state.power_supply_health = 0;     // unknown
  batt_state.power_supply_technology = 3; // LiPo
  batt_state.present = 1;                 // battery present

  batt_state.location = "Toulouse";        // unit location
  batt_state.serial_number = "ABC_0001";  // unit serial number

}

void loop()
{
  // Battery voltage
  // I wired resistors to step down the lipo voltage 50%
  // so multiply by 2 to get back up to obeserved voltage
  // I did not connect to cells, simply to the output voltage.
  double lipoVoltage = analogRead(battPin) * K * 2;
  batt_state.voltage = (float)lipoVoltage;

  // Update battery health.
  if (batt_state.voltage > CELLS * 4.2)
    batt_state.power_supply_health = 4; // overvoltage
  else if (batt_state.voltage < CELLS * 3.0)
    batt_state.power_supply_health = 3; // dead
  else
    batt_state.power_supply_health = 1; // good

  // Publish data to ROSSERIAL.
  batteryState.publish( &batt_state );
  nh.spinOnce();
  delay(1000);
}
