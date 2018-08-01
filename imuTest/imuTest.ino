#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle nh;
geometry_msgs::Twist position_msg;
ros::Publisher chatter("position", &position_msg);

void setup()
{
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  nh.initNode();
  nh.advertise(chatter);

  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop()
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  position_msg.angular.x = event.orientation.x;
  position_msg.angular.y = event.orientation.y;
  position_msg.angular.z = event.orientation.z;
  
  position_msg.linear.x = 0;
  position_msg.linear.y = 0;
  position_msg.linear.z = 0;
  
  chatter.publish(&position_msg);
  
  nh.spinOnce();
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
