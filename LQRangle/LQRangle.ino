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
#include <std_msgs/Empty.h>
#include <RDRONE/Throttle.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>


/* 
 * Controller Class
 */
 
class Controller {
  
  private: 
  int throttleFL;
  int throttleFR;
  int throttleRL;
  int throttleRR;
  Servo servoFL;
  Servo servoRL;
  Servo servoFR;
  Servo servoRR;
  
  public:
  Controller() {
    throttleFL = 1200;
    throttleFR = 1200;
    throttleRL = 1200;
    throttleRR = 1200;
  }
  void init() {
  servoFL.attach(6);
  servoRL.attach(7);
  servoFR.attach(8);
  servoRR.attach(9);
}
  
   int getThrottleFL() {
    return throttleFL;
  }

  int getThrottleFR() {
    return throttleFR;
  }

  int getThrottleRL() {
    return throttleRL;
  }

  int getThrottleRR() {
    return throttleRR;
  }
  
  void processMsg( const RDRONE::Throttle& throttle_msg) {
    throttleFL = throttle_msg.FL;
    throttleFR = throttle_msg.FR;
    throttleRL = throttle_msg.RL;
    throttleRR = throttle_msg.RR;

  }
  
  void writeThrottle(const RDRONE::Throttle& throttle_msg) {
    servoFL.writeMicroseconds(throttle_msg.FL);
    servoFR.writeMicroseconds(throttle_msg.FR);
    servoRR.writeMicroseconds(throttle_msg.RR);
    servoRL.writeMicroseconds(throttle_msg.RL);
  }
  
};

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
float K[4][6] = {{-55.6978,  -37.3957,   55.4299,   36.7247,   28.7811,   67.8958},
                    {47.2508,   29.5039,   48.2203,   31.9752,  -39.5835,  -95.5539},
                  {-52.6087,  -34.0078,  -51.7228,  -31.7588,  -36.4119,  -87.0427},
                 {43.5596,   30.0663,  -43.8984,  -30.9278,   35.7627,   86.9344}};

/* 
 * setup publisher and subscriber
 */
ros::NodeHandle_<ArduinoHardware,1,2,200,150> nh;
ros::Subscriber<RDRONE::Throttle> sub("throttle_cmd", &messageCb);
geometry_msgs::Twist position_msg;
RDRONE::Throttle ctrl;
ros::Publisher chatter("position", &position_msg);
ros::Publisher spatter("control_action", &ctrl);

/* 
 * setup the node 
 */
void setup()
{
  controller.init();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  nh.advertise(spatter);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //Serial.println("initialized imu");
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
 if (t1-t0 >= rate) {
   //Serial.println("running feedback");
    /* Get a new sensor event */
    
    sensors_event_t event;
    bno.getEvent(&event);
    
    r = -1.0*event.orientation.z*(3.14159/180.0);
    g = -1.0*event.orientation.y*(3.14159/180.0);
    th = -1.0*event.orientation.x*(3.14159/180.0);
    if (th <= -3.14159) {
      th = 2*3.14159 + th;
    }
    
    rdot = (r - rp)/(dt);
    gdot = (g - gp)/(dt);
    thdot = (th - thp)/(dt);
   
    ctrl.FL = controller.getThrottleFL() + K[0][0]*r +  K[0][1]* rdot +K[0][2] *g + K[0][3] * gdot + K[0][4] * th + K[0][5] * thdot; 
    ctrl.FR = controller.getThrottleFR() + K[1][0]*r +  K[1][1]* rdot +K[1][2] *g + K[1][3] * gdot + K[1][4] * th + K[1][5] * thdot; 
    ctrl.RL = controller.getThrottleRL() + K[2][0]*r +  K[2][1]* rdot +K[2][2] *g + K[2][3] * gdot + K[2][4] * th + K[2][5] * thdot; 
    ctrl.RR = controller.getThrottleRR() + K[3][0]*r +  K[3][1]* rdot +K[3][2] *g + K[3][3] * gdot + K[3][4] * th + K[3][5] * thdot; 
   
    
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
    
    spatter.publish(&ctrl);
    chatter.publish(&position_msg);
    t0 = millis();
    t1 = millis();
    
  } else {
    t1 = millis();
  }
  nh.spinOnce();
}

