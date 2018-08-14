/*
 * Object-Oriented example publisher-subscriber model for ROS
 */

/* 
 * defining dependencies 
 */
#include <ros.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Servo.h>
#include <RDRONE/Throttle.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

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
    throttle = 1200;
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
    throttle = throttle_msg.FL;
  }
  
  void writeThrottle(const RDRONE::Throttle& throttle_msg) {
    servoFL.writeMicroseconds(throttle_msg.FL);
    servoFR.writeMicroseconds(throttle_msg.FR);
    servoRR.writeMicroseconds(throttle_msg.RR);
    servoRL.writeMicroseconds(throttle_msg.RL);
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
geometry_msgs::Twist position_msg;
RDRONE::Throttle ctrl;
ros::Publisher chatter("position", &position_msg);

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long t0;
unsigned long t1;
unsigned long rate = 40;
float dt = 1.0/25.0;

float r;
float g;
float th;
float rp;
float gp;
float thp;

float rdot;
float gdot;
float thdot;

/* 
 * setup the node 
 */
void setup()
{
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("initialized imu");
  controller.init();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  delay(1000);
  bno.setExtCrystalUse(true);
  rdot = 0;
  gdot = 0;
  thdot = 0;
  rp = 0;
  gp = 0;
  thp = 0;
  t0 = millis();
  t1 = millis();
}

/* 
 * run update code
 */
void loop()
{
 if (t1-t0 >= 500) {
   Serial.println("running feedback");
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    
    r = -event.orientation.z*(3.14159/180.0);
    g = -event.orientation.y*(3.14159/180.0);
    th = -event.orientation.x*(3.14159/180.0);
    
    rdot = (r - rp)/(dt);
    gdot = (g - gp)/(dt);
    thdot = (th - thp)/(dt);
    
    ctrl.FL = controller.getThrottle() - 78.0*r - 44.0 * rdot + 78.5*g + 43.7 * gdot + 28.5 * th + 67.7 * thdot; 
    ctrl.FR = controller.getThrottle() + 67.9*r + 35.1 * rdot + 67.9*g + 37.6 * gdot - 39.6 * th - 95.6 * thdot;
    ctrl.RL = controller.getThrottle() - 74.3*r - 40.1 * rdot - 73.4*g - 37.9 * gdot - 36.3 * th - 86.9 * thdot;
    ctrl.RR = controller.getThrottle() + 61.5*r + 35.5 * rdot - 61.8*g - 36.4 * gdot + 35.9 * th + 87.0 * thdot;
    
    rp = r;
    gp = g;
    thp = th;
    
    controller.writeThrottle(ctrl);
    
    position_msg.angular.x = r;
    position_msg.angular.y = g;
    position_msg.angular.z = th;
  
    position_msg.linear.x = rdot;
    position_msg.linear.y = gdot;
    position_msg.linear.z = thdot;
    
    chatter.publish(&position_msg);
    t0 = millis();
    t1 = millis();
    
  } else {
    t1 = millis();
  }
  nh.spinOnce();
}

