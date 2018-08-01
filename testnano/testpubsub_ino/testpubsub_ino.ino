/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

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

Controller controller;

void messageCb(const std_msgs::Empty& toggle_msg) {
  controller.processMsg(toggle_msg);
}

ros::Subscriber<std_msgs::Empty> sub("increment", &messageCb);
std_msgs::Int32 int_msg;
ros::Publisher chatter("count", &int_msg);


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop()
{
  int_msg.data = controller.getCount();
  chatter.publish(int_msg);
  nh.spinOnce();
  delay(1000);
}

