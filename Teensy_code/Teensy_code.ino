#include <ArduinoEigenSparse.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigen.h>

#include "interpo.h"
#include "InverseKin.h"
#include "ForwKin.h"
#include "IK.h"

InverseKinematics InKin;

//Some variables
//Joint Angle Variables for SetNextpathPoint Mode
double theta1[6];
double theta2[6];
double theta3[6];
double dt_theta1[6];
double dt_theta2[6];
double dt_theta3[6];

// Dynamixel
#include "MyDynamixel.h"
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

//ROS
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <my_message/LegPath.h>
#include <my_message/thetaMessage.h>

#include <hexapod_ros/EffectorTargets.h>
#include <hexapod_ros/HexapodCommands.h>

ros::NodeHandle nh;

//LOG data to file publisher
std_msgs::String logdata;
ros::Publisher pub_log("LOGDATA", &logdata);


//Current position publish
Eigen::Vector3d effector_current_positions[6];
hexapod_ros::EffectorTargets effector_current_positions_msg;
ros::Publisher pub_effector_positions("effector_current_positions", &effector_current_positions_msg);

void push_log(char* message);
void push_effector_positions();

IK ik(&dxl, push_log);


void push_effector_positions()
{
  for (int leg_id=0; leg_id<6; leg_id++)
  {
    Eigen::Vector3d pos = ik.leg_to_robot_space(effector_current_positions[leg_id], leg_id);
    effector_current_positions_msg.targets[leg_id].data[0] = pos[0];
    effector_current_positions_msg.targets[leg_id].data[1] = pos[1];
    effector_current_positions_msg.targets[leg_id].data[2] = pos[2];
  }
  
  pub_effector_positions.publish(&effector_current_positions_msg);
}

void push_log(char* message)
{
  logdata.data = message;
  pub_log.publish(&logdata);
}

// ** ROS callback & subscriber **


// Get effector targets from message
Eigen::Vector3d effector_targets[6];
bool is_swinging[6];
void targets_cb(const hexapod_ros::EffectorTargets& msg)
{
  for (int i=0; i<6; i++)
  {
    effector_targets[i] = (Eigen::Vector3f {msg.targets[i].data}).cast<double>();
    is_swinging[i] = msg.swinging[i];
  }
}
ros::Subscriber<hexapod_ros::EffectorTargets> rosSubEffectorTargets("effector_targets", targets_cb);


double move_speed = 0;
void commands_cb(const hexapod_ros::HexapodCommands& msg)
{
  move_speed = msg.speed;
}
ros::Subscriber<hexapod_ros::HexapodCommands> rosSubCommands("hexapod_command_data", commands_cb);

//mode select subscriber
uint8_t IDS[18] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
int mode = -1;
int startUp = -1;
void modeSelect_cb(const std_msgs::Int32& msg)
{
  if(msg.data == -1)
  {
    startUp = -1;
    mode = -1;
    logdata.data = "Torque off";
    
    // Disable servo torque
    dxl.SyncDisableTorque(IDS, 18);
  }
  else if(msg.data == 0)
  {
    logdata.data = "Startup";
    startUp = 0;
  }
  else
  {
    logdata.data = "Other Mode";
    mode = msg.data;
  }

  // LOG PUBLISH
  pub_log.publish(&logdata);
}
ros::Subscriber<std_msgs::Int32> rosSubModeSelect("hexapod_mode", modeSelect_cb);


void setup() 
{
 Serial.begin(9600);
 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, HIGH);

  // ROS setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(rosSubEffectorTargets);
  nh.subscribe(rosSubCommands);
  nh.subscribe(rosSubModeSelect);
  nh.advertise(pub_log);
  nh.advertise(pub_effector_positions);
}

char dataStr[100] = "";
char buff[7];
long currentmillis = 0;
long prevmillis = millis();
float x=0.0;

//global variables for setnextpathpoint
long curtime = 0;
long prevtime = 0;

int stepStartFlag = 0;
int kinematicModeStartFlag = 0;

//Function prototype
void SetAngles(double* th1,double* th2,double* th3 ,double spd1=-1,double spd2=-1, double spd3=-1);

Eigen::Vector3d targets[6] = {
  {207.846, -120.000, -140},
  {0.000, -240.000, -140},
  {-207.846, -120.000, -140},
  {-207.846, 120.000, -140},
  {-0.000, 240.000, -140},
  {207.846, 120.000, -140},
};

void loop()
{
  nh.spinOnce();
  currentmillis = millis();

  // If servos are sent updates at too high a frequency jitter occurs
  if(currentmillis - prevmillis >= 100)
  {
    char msg[50];
    sprintf(msg,"dt %ld", currentmillis - prevmillis);
//    push_log(msg);
    prevmillis = currentmillis;

    //On Startup
      for (int leg_id=0; leg_id<6; leg_id++)
      {
        effector_current_positions[leg_id] = ik.solve_current_position(leg_id);
      }
    if(startUp == 0)
    { 


      sprintf(msg,"move speed: %f", move_speed);
      push_log(msg);
      ik.set_final_targets(effector_targets, is_swinging);
      ik.solve_next_moves(theta1[0], theta2[0], theta3[0], dt_theta1[0], dt_theta2[0], dt_theta3[0], move_speed, 0);
      ik.solve_next_moves(theta1[1], theta2[1], theta3[1], dt_theta1[1], dt_theta2[1], dt_theta3[1], move_speed, 1);
      ik.solve_next_moves(theta1[2], theta2[2], theta3[2], dt_theta1[2], dt_theta2[2], dt_theta3[2], move_speed, 2);
      ik.solve_next_moves(theta1[3], theta2[3], theta3[3], dt_theta1[3], dt_theta2[3], dt_theta3[3], move_speed, 3);
      ik.solve_next_moves(theta1[4], theta2[4], theta3[4], dt_theta1[4], dt_theta2[4], dt_theta3[4], move_speed, 4);
      ik.solve_next_moves(theta1[5], theta2[5], theta3[5], dt_theta1[5], dt_theta2[5], dt_theta3[5], move_speed, 5);

      SetAngles(theta1, theta2, theta3, dt_theta1, dt_theta2, dt_theta3);
      push_effector_positions();
    }
  }
}


void SetAngles(double* th1, double* th2, double* th3, double* dt_th1,double* dt_th2, double* dt_th3)
{
  double angles[18];
  double speeds[18];
  
  for(int leg_id = 0; leg_id<6; leg_id++)
  {
    angles[leg_id*3] = -th1[leg_id];
    angles[leg_id*3+1] = th2[leg_id];
    angles[leg_id*3+2] = th3[leg_id];

    speeds[leg_id*3] = dt_th1[leg_id];
    speeds[leg_id*3+1] = dt_th2[leg_id];
    speeds[leg_id*3+2] = dt_th3[leg_id];
  }
  dxl.SyncMove(IDS, angles, speeds, 18);
}


int ConstrainCheck01(float* th1,float* th2,float* th3)
{
  int LegitMove = 1;

  for(int i = 0;i<6;i++)
  {
    if((th3[i] > 0.0) || (th3[i] < -150.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th2[i] > 60.0/180.0*M_PI) || (th2[i] < -90.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th1[i] > 50.0/180.0*M_PI) || (th1[i] < -50.0/180.0*M_PI))
    {
      LegitMove = 0;
    }
  }
  return LegitMove;
}

void ConstrainCheck2(float* th1,float* th2,float* th3)
{
  static float prevTh1[6] = {0,0,0,0,0,0};
  static float prevTh2[6] = {0,0,0,0,0,0};
  static float prevTh3[6] = {0,0,0,0,0,0};
  float px = 0, py = 0, pz = 0;

  int LegitMove = 1;

  for(int i = 0;i<6;i++)
  {
    if((th3[i] > 0.0) || (th3[i] < -150.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th2[i] > 60.0/180.0*M_PI) || (th2[i] < -90.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th1[i] > 50.0/180.0*M_PI) || (th1[i] < -50.0/180.0*M_PI))
    {
      LegitMove = 0;
    }
  }

  if(LegitMove == 1)
  {
    for(int i = 0;i<6;i++)
    {
      prevTh1[i] = th1[i];
      prevTh2[i] = th2[i];
      prevTh3[i] = th3[i];
    }
  }
  else if(LegitMove == 0)
  {
    for(int i = 0;i<6;i++)
    {
      th1[i] = prevTh1[i];
      th2[i] = prevTh2[i];
      th3[i] = prevTh3[i];

      FK03_inbody(px,py,pz, th1[i],th2[i],th3[i],i);
      InKin.IK(&th1[i],&th2[i],&th3[i],px,   py,    pz,i,0,0,0,0,0);
    }
  }
}
