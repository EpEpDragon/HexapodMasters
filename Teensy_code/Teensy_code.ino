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


//Joint Angle Variables for SetAngle Mode
float A_theta1[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float A_theta2[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float A_theta3[6]={-1.5, -1.5, -1.5, -1.5, -1.5, -1.5};


//Joint Angle Variables for Teleop demo Mode
float T_theta1[6];
float T_theta2[6];
double T_theta3[6];


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

ros::NodeHandle nh;

//LOG data to file publisher
std_msgs::String logdata;
ros::Publisher pub_log("LOGDATA", &logdata);

//Feet step
std_msgs::Float32 FeetOnFloor;
ros::Publisher pub_FeetOnFloorFlag("/FeetOnFloorFlag", &FeetOnFloor); 


void push_log(char* message)
{
  logdata.data = message;
  pub_log.publish(&logdata);
}

// ** ROS callback & subscriber **

//Change angle of joints subscriber
void angle_command_cb( const my_message::thetaMessage& msg)
{
  A_theta1[0] = msg.th1_1;
  A_theta1[1] = msg.th1_2;
  A_theta1[2] = msg.th1_3;
  A_theta1[3] = msg.th1_4;
  A_theta1[4] = msg.th1_5;
  A_theta1[5] = msg.th1_6;

  A_theta2[0] = msg.th2_1;
  A_theta2[1] = msg.th2_2;
  A_theta2[2] = msg.th2_3;
  A_theta2[3] = msg.th2_4;
  A_theta2[4] = msg.th2_5;
  A_theta2[5] = msg.th2_6;

  A_theta3[0] = msg.th3_1;
  A_theta3[1] = msg.th3_2;
  A_theta3[2] = msg.th3_3;
  A_theta3[3] = msg.th3_4;
  A_theta3[4] = msg.th3_5;
  A_theta3[5] = msg.th3_6;
  
}
ros::Subscriber<my_message::thetaMessage> rosSub("/simple_hexapod/Th_position_controller/command", angle_command_cb);


//Sub to teleop_keyboard for demo stuff
int tx=0,ty=0,tz=0;
float troll=0,tpitch=0,tyaw=0;
void teleop_cb(const geometry_msgs::Twist& msg)
{
  //lean x direction
  if (msg.linear.x > 0 && msg.linear.x <= 0.5)
  {
    tx = tx + 1;
  }
  else if (msg.linear.x < 0 && msg.linear.x >= -0.5)
  {
    tx = tx - 1;
  }
  //lean y direction
  if (msg.linear.y > 0 && msg.linear.y <= 0.5)
  {
    ty = ty + 1;
  }
  else if (msg.linear.y < 0 && msg.linear.y >= -0.5)
  {
    ty = ty - 1;
  }
  //Change height
  if (msg.linear.z > 0)
  {
    tz = tz + 1;
  }
  else if (msg.linear.z < 0)
  {
    tz = tz - 1;
  }
  //Yaw movement
  if (msg.angular.z > 0)
  {
    tyaw = tyaw + 0.25;
  }
  else if (msg.angular.z < 0)
  {
    tyaw = tyaw - 0.25;
  }
  //Roll movement
  if (msg.linear.x > 0.5)
  {
    troll = troll + 0.25;
  }
  else if (msg.linear.x < -0.5)
  {
    troll = troll - 0.25;
  }
  //Pitch movement
  if (msg.linear.y > 0.5)
  {
    tpitch = tpitch + 0.25;
  }
  else if (msg.linear.y < -0.5)
  {
    tpitch = tpitch - 0.25;
  }
}
ros::Subscriber<geometry_msgs::Twist> rosSubTeleop("/cmd_vel", teleop_cb);

// Get effector targets from message
Eigen::Vector3d effector_targets[6];
void targets_cb(const hexapod_ros::EffectorTargets& msg)
{
  for (int i=0; i<6; i++)
  {
    effector_targets[i] = (Eigen::Vector3f {msg.targets[i].data}).cast<double>();
  }
}
ros::Subscriber<hexapod_ros::EffectorTargets> rosSubEffectorTargets("effector_targets", targets_cb);

//Leg Path subsciber
#define Pathsize 7
float XPath[6][Pathsize*2-2];
float YPath[6][Pathsize*2-2];
float ZPath[6][Pathsize*2-2];
float TurnPath[Pathsize*2-2];
float dt=0;
int startSetPath = 0;
int standBegin = 0;
void legpath_cb(const my_message::LegPath& msg)
{

  for (int col=0;col<(Pathsize*2-2);col++)
  {
    XPath[0][col]=msg.PathL0x[col];
    XPath[1][col]=msg.PathL1x[col];
    XPath[2][col]=msg.PathL2x[col];
    XPath[3][col]=msg.PathL3x[col];
    XPath[4][col]=msg.PathL4x[col];
    XPath[5][col]=msg.PathL5x[col];

    YPath[0][col]=msg.PathL0y[col];
    YPath[1][col]=msg.PathL1y[col];
    YPath[2][col]=msg.PathL2y[col];
    YPath[3][col]=msg.PathL3y[col];
    YPath[4][col]=msg.PathL4y[col];
    YPath[5][col]=msg.PathL5y[col];

    ZPath[0][col]=msg.PathL0z[col];
    ZPath[1][col]=msg.PathL1z[col];
    ZPath[2][col]=msg.PathL2z[col];
    ZPath[3][col]=msg.PathL3z[col];
    ZPath[4][col]=msg.PathL4z[col];
    ZPath[5][col]=msg.PathL5z[col];

    TurnPath[col]=msg.PathAng[col];

    dt = msg.DT;
  }

  startSetPath = 1;
  standBegin = 1; 

}
ros::Subscriber<my_message::LegPath> rosSubPATH("/simple_hexapod/Legs_paths", legpath_cb);

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

//Pitch roll input subscriber
float pitchInput = 0.0;
float rollInput = 0.0;
void PitchRollInput_cb(const geometry_msgs::Vector3& msg)
{
    pitchInput = msg.x;
    rollInput = msg.y;
}
ros::Subscriber<geometry_msgs::Vector3> rosSubPitchRollInput("/RollPitch_input", PitchRollInput_cb);

void setup() 
{
 Serial.begin(9600);
 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, HIGH);

  // ROS setup
  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();                             // init ROS
  // Effector target subscriber
  nh.subscribe(rosSubEffectorTargets);
  //angle subscriber
  nh.subscribe(rosSub);
  //teleop_keyboard subscriber
  nh.subscribe(rosSubTeleop);
  //Leg Path subsciber
  nh.subscribe(rosSubPATH);
  //mode select subscriber
  nh.subscribe(rosSubModeSelect);
  //Pitch roll input subscriber
  nh.subscribe(rosSubPitchRollInput);
  //data log publisher
  nh.advertise(pub_log);
  //Feet step
  nh.advertise(pub_FeetOnFloorFlag);
  //broadcaster.init(nh);       // set up broadcaster fot tf
  //nh.advertise(pub_eye);   // advertise eye topic
 
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

IK ik(&dxl, push_log);


void loop()
{
  nh.spinOnce();
  currentmillis = millis();

  if(currentmillis - prevmillis >= 10)
  {  
    prevmillis = currentmillis;

    //On Startup
    if(startUp == 0)
    {
      // static long startUp_startTime = millis();
      ik.set_final_targets(effector_targets);
      ik.solve_next_angles(theta1[0], theta2[0], theta3[0], 0);
      ik.solve_next_angles(theta1[1], theta2[1], theta3[1], 1);
      ik.solve_next_angles(theta1[2], theta2[2], theta3[2], 2);
      ik.solve_next_angles(theta1[3], theta2[3], theta3[3], 3);
      ik.solve_next_angles(theta1[4], theta2[4], theta3[4], 4);
      ik.solve_next_angles(theta1[5], theta2[5], theta3[5], 5);
      
      SetAngles(theta1,theta2,theta3,5,5,5);
    }

    //Teleop demo Mode
    if(mode == 1 && startUp == 1)
    {
//      stepStartFlag = 0;
//      static elapsedMillis kinematicModeStartTimer;
//
//      if(kinematicModeStartFlag == 0)
//      {
//        kinematicModeStartFlag = 1;
//        kinematicModeStartTimer = 0;
//      }
//
//      if(kinematicModeStartTimer <= 1000)
//      {
//        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,1000,troll,tpitch,tyaw,0);
//        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,1000,troll,tpitch,-tyaw,0);
//        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,1000,troll,tpitch,tyaw,0);
//        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,1000,troll,tpitch,-tyaw,0);
//        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,1000,troll,tpitch,tyaw,0);
//        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,1000,troll,tpitch,-tyaw,0);
//        if(ConstrainCheck01(T_theta1,T_theta2,T_theta3))
//        {
//          SetAngles(T_theta1,T_theta2,T_theta3,0,0,0);
//        }
//      }
//      else
//      {
//        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,0,troll,tpitch,tyaw,0);
//        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,0,troll,tpitch,-tyaw,0);
//        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,0,troll,tpitch,tyaw,0);
//        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,0,troll,tpitch,-tyaw,0);
//        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,0,troll,tpitch,tyaw,0);
//        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,0,troll,tpitch,-tyaw,0);
//        if(ConstrainCheck01(T_theta1,T_theta2,T_theta3))
//        {
//          SetAngles(T_theta1,T_theta2,T_theta3,0,0,0);
//        }
//      }
    }

    //SetAngle Mode
    else if(mode == 2 && startUp == 1)
    {
//      stepStartFlag = 0;
//      kinematicModeStartFlag = 0;
//      
//      A_theta1[0] = 0;
//      A_theta2[0] = 0;
//      A_theta3[0] = -0;
//
//      float px = 0, py = 0, pz = 0;
//      
//      FK03_inbody(px,py,pz, A_theta1[0],A_theta2[0],A_theta3[0],0);
//      InKin.IK(&A_theta1[0],&A_theta2[0],&A_theta3[0],px,   py,    pz,0,0,0,0,0,0);
//
//      FK03_inbody(px,py,pz, A_theta1[1],A_theta2[1],A_theta3[1],1);
//      InKin.IK(&A_theta1[1],&A_theta2[1],&A_theta3[1],px,   py,    pz,1,0,0,0,0,0);
//
//      FK03_inbody(px,py,pz, A_theta1[2],A_theta2[2],A_theta3[2],2);
//      InKin.IK(&A_theta1[2],&A_theta2[2],&A_theta3[2],px,   py,    pz,2,0,0,0,0,0);
//
//      FK03_inbody(px,py,pz, A_theta1[3],A_theta2[3],A_theta3[3],3);
//      InKin.IK(&A_theta1[3],&A_theta2[3],&A_theta3[3],px,   py,    pz,3,0,0,0,0,0);
//
//      FK03_inbody(px,py,pz, A_theta1[4],A_theta2[4],A_theta3[4],4);
//      InKin.IK(&A_theta1[4],&A_theta2[4],&A_theta3[4],px,   py,    pz,4,0,0,0,0,0);
//
//      FK03_inbody(px,py,pz, A_theta1[5],A_theta2[5],A_theta3[5],5);
//      InKin.IK(&A_theta1[5],&A_theta2[5],&A_theta3[5],px,   py,    pz,5,0,0,0,0,0);
//
//      if(ConstrainCheck01(A_theta1,A_theta2,A_theta3))
//      {
//        SetAngles(A_theta1,A_theta2,A_theta3,20,20,20);
//      }
//      //mode = 2;
    }
    
    //SetNextpathPoint Mode
    else if(mode == 3 && startSetPath == 1 && startUp == 1)
    {

    }
  }
}


void SetAngles(double* th1, double* th2, double* th3, double spd1,double spd2, double spd3)
{
  double angles[18];
  double speeds[18];
  
  for(int leg_id = 0; leg_id<6; leg_id++)
  {
    angles[leg_id*3] = -th1[leg_id];
    angles[leg_id*3+1] = th2[leg_id];
    angles[leg_id*3+2] = th3[leg_id];

    speeds[leg_id*3] = spd1;
    speeds[leg_id*3+1] = spd2;
    speeds[leg_id*3+2] = spd3;
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
