/*
 * Object-Oriented example publisher-subscriber model for ROS
 */

/* 
 * defining dependencies 
 */
#include <ros.h>
#include <Servo.h>
#include <RDRONE/Throttle.h>
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
  Servo servoFL;
  Servo servoRL;
  Servo servoFR;
  Servo servoRR;
  
  public:
  Controller() {
    throttle = 0;
  }
  void init() {
  servoFL.attach(9);
  servoRL.attach(8);
  servoFR.attach(7);
  servoRR.attach(6);
}
  
  int getThrottle() {
    return throttle;
  }
  
  void processMsg( const RDRONE::Throttle& throttle_msg) {
    servoFL.writeMicroseconds(throttle_msg.FL);
    servoFR.writeMicroseconds(throttle_msg.FR);
    servoRR.writeMicroseconds(throttle_msg.RR);
    servoRL.writeMicroseconds(throttle_msg.RL);
    throttle = throttle_msg.FL;
  }
  
};
/*
 * initialize controller
 */
Controller controller;

/*
 * define subscriber message callback
 */
void messageCb(const RDRONE::Throttle& throttle_msg) {
  controller.processMsg(throttle_msg);
}

/* 
 * setup publisher and subscriber
 */
ros::NodeHandle nh;
ros::Subscriber<RDRONE::Throttle> sub("throttle_cmd", &messageCb);
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

