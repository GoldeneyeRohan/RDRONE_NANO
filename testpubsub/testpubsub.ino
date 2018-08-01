/*
 * Object-Oriented example publisher-subscriber model for ROS
 */

/* 
 * defining dependencies 
 */
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

/* 
 * Controller Class
 */
 
class Controller {
  
  private: 
  int count;
  
  public:
  Controller() {
    count = 0;
  }
  
  int getCount() {
    return count;
  }
  
  void processMsg( const std_msgs::Empty& toggle_msg) {
    count ++;
  }
  
};
/*
 * initialize controller
 */
Controller controller;
unsigned long t0;
unsigned long t1;
/*
 * define subscriber message callback
 */
void messageCb(const std_msgs::Empty& toggle_msg) {
  t0 = millis();
  t1 = millis();
  analogWrite(7,20);
  while (t1 - t0 < 1000) {
  }
  analogWrite(7,0);
}

/* 
 * setup publisher and subscriber
 */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Empty> sub("throttle", &messageCb);
std_msgs::Int32 int_msg;
ros::Publisher chatter("count", &int_msg);

/* 
 * setup the node 
 */
void setup()
{
  pinMode(7,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

/* 
 * run update code
 */
void loop()
{
  int_msg.data = controller.getCount();
  chatter.publish(&int_msg);
  nh.spinOnce();
  delay(1000);
}

