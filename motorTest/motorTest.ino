/*
 * Object-Oriented example publisher-subscriber model for ROS
 */

/* 
 * defining dependencies 
 */
#include <ros.h>
#include <Servo.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>

/* 
 * Controller Class
 */
 
class Controller {
  
  private: 
  int throttle;
  unsigned long t0;
  unsigned long t1;
  Servo servo;
  
  public:
  Controller() {
    throttle = 0;
  }
  void init() {
  servo.attach(9);}
  
  int getThrottle() {
    return throttle;
  }
  
  void processMsg( const std_msgs::Int32& throttle_msg) {
    throttle = throttle_msg.data;
    t0 = millis();
    t1 = millis();
    //analogWrite(7,throttle);
    servo.writeMicroseconds(throttle);
    //while (t1 - t0 < 3000) {
    //  t1 = millis();
    //}
    //servo.write(0);
    //analogWrite(7,0);
  }
  
};
/*
 * initialize controller
 */
Controller controller;

/*
 * define subscriber message callback
 */
void messageCb(const std_msgs::Int32& throttle_msg) {
  controller.processMsg(throttle_msg);
}

/* 
 * setup publisher and subscriber
 */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> sub("throttle_cmd", &messageCb);
std_msgs::Int32 int_msg;
ros::Publisher chatter("throttle", &int_msg);

/* 
 * setup the node 
 */
void setup()
{
  controller.init();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

/* 
 * run update code
 */
void loop()
{
  int_msg.data = controller.getThrottle();
  chatter.publish(&int_msg);
  nh.spinOnce();
  delay(1000);
}

