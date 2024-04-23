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
float theta1[6];
float theta2[6];
float theta3[6];


//Joint Angle Variables for SetAngle Mode
float A_theta1[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float A_theta2[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float A_theta3[6]={-1.5, -1.5, -1.5, -1.5, -1.5, -1.5};


//Joint Angle Variables for Teleop demo Mode
float T_theta1[6];
float T_theta2[6];
float T_theta3[6];


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

//Feedback publishers
std_msgs::String motor_feedback[18];
ros::Publisher pub_m_feed0("motor_feedback11", &motor_feedback[0]);
ros::Publisher pub_m_feed1("motor_feedback21", &motor_feedback[1]);
ros::Publisher pub_m_feed2("motor_feedback31", &motor_feedback[2]);
ros::Publisher pub_m_feed3("motor_feedback12", &motor_feedback[3]);
ros::Publisher pub_m_feed4("motor_feedback22", &motor_feedback[4]);
ros::Publisher pub_m_feed5("motor_feedback32", &motor_feedback[5]);
ros::Publisher pub_m_feed6("motor_feedback13", &motor_feedback[6]);
ros::Publisher pub_m_feed7("motor_feedback23", &motor_feedback[7]);
ros::Publisher pub_m_feed8("motor_feedback33", &motor_feedback[8]);
ros::Publisher pub_m_feed9("motor_feedback14", &motor_feedback[9]);
ros::Publisher pub_m_feed10("motor_feedback24", &motor_feedback[10]);
ros::Publisher pub_m_feed11("motor_feedback34", &motor_feedback[11]);
ros::Publisher pub_m_feed12("motor_feedback15", &motor_feedback[12]);
ros::Publisher pub_m_feed13("motor_feedback25", &motor_feedback[13]);
ros::Publisher pub_m_feed14("motor_feedback35", &motor_feedback[14]);
ros::Publisher pub_m_feed15("motor_feedback16", &motor_feedback[15]);
ros::Publisher pub_m_feed16("motor_feedback26", &motor_feedback[16]);
ros::Publisher pub_m_feed17("motor_feedback36", &motor_feedback[17]);

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
Eigen::Vector3f effector_targets[6];
void targets_cb(const hexapod_ros::EffectorTargets& msg)
{
  for (int i=0; i<6; i++)
  {
    effector_targets[i] = Eigen::Vector3f {msg.targets[i].data};
//    char temp[50];
//    sprintf(temp, "Targets: [%.2f, %.2f, %.2f]", effector_targets[i][0], effector_targets[i][1], effector_targets[i][2]);
//    logdata.data = temp; 
//    pub_log.publish(&logdata);
  }
//  logdata.data = "----------------"; 
//  pub_log.publish(&logdata);
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
  //feedback publisher
  nh.advertise(pub_m_feed0);
  nh.advertise(pub_m_feed1);
  nh.advertise(pub_m_feed2);
  nh.advertise(pub_m_feed3);
  nh.advertise(pub_m_feed4);
  nh.advertise(pub_m_feed5);
  nh.advertise(pub_m_feed6);
  nh.advertise(pub_m_feed7);
  nh.advertise(pub_m_feed8);
  nh.advertise(pub_m_feed9);
  nh.advertise(pub_m_feed10);
  nh.advertise(pub_m_feed11);
  nh.advertise(pub_m_feed12);
  nh.advertise(pub_m_feed13);
  nh.advertise(pub_m_feed14);
  nh.advertise(pub_m_feed15);
  nh.advertise(pub_m_feed16);
  nh.advertise(pub_m_feed17);
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
void SetAngles(float* th1,float* th2,float* th3 ,float spd1=-1,float spd2=-1, float spd3=-1);

Eigen::Vector3f targets[6] = {
  {283.71,   0.0,    -140},
  {141.855,  -245.7, -140},
  {-141.855, -245.7, -140},
  {-283.71,  0.0,    -140},
  {-141.855, 245.7,  -140},
  {141.855,  245.7,  -140}
};

IK ik(&dxl, push_log);

void loop()
{
  nh.spinOnce();
  
  currentmillis = millis();

  if(currentmillis - prevmillis >= 10)
  {
      ik.test();
//    float theta[6];
//    theta[0] = dxl.PresentPos(0)*180/PI;
//    theta[1] = dxl.PresentPos(1)*180/PI;
//    theta[2] = dxl.PresentPos(2)*180/PI;
//    theta[3] = dxl.PresentPos(9)*180/PI;
//    theta[4] = dxl.PresentPos(12)*180/PI;
//    theta[5] = dxl.PresentPos(15)*180/PI;
//    char msg[50];
//    sprintf(msg, "LOG: theta1: %.2f, theta2: %.2f, theta3 %.2f", theta[0], theta[1], theta[2], theta[3],theta[4],theta[5]);
//    push_log(msg);
    
    prevmillis = currentmillis;

    //On Startup
    if(startUp == 0)
    {
      static long startUp_startTime = millis();

      ik.set_final_targets(targets);
      ik.solve_next_angles(theta1[0], theta2[0], theta3[0],0);
      ik.solve_next_angles(theta1[1], theta2[1], theta3[1],1);
      ik.solve_next_angles(theta1[2], theta2[2], theta3[2],2);
      ik.solve_next_angles(theta1[3], theta2[3], theta3[3],3);
      ik.solve_next_angles(theta1[4], theta2[4], theta3[4],4);
      ik.solve_next_angles(theta1[5], theta2[5], theta3[5],5);
      
      char msg[50];
      sprintf(msg, "LOG: theta1: %.2f, theta2: %.2f, theta3 %.2f", theta1[0], theta2[0], theta3[0]);
      push_log(msg);
      // char msg[50];
      // sprintf(msg, "LOG: theta1: %.2f, theta2: %.2f, theta3 %.2f", theta1[0], theta2[0], theta3[0]);
      // logdata.data = msg;
      // pub_log.publish(&logdata);

      InKin.IK(&theta1[0],&theta2[0],&theta3[0],283.71,   0.0,    -140,0,0,0,0,0,0);
      InKin.IK(&theta1[1],&theta2[1],&theta3[1],141.855,  -245.7, -140,1,0,0,0,0,0);
      InKin.IK(&theta1[2],&theta2[2],&theta3[2],-141.855, -245.7, -140,2,0,0,0,0,0);
      InKin.IK(&theta1[3],&theta2[3],&theta3[3],-283.71,  0.0,    -140,3,0,0,0,0,0);
      InKin.IK(&theta1[4],&theta2[4],&theta3[4],-141.855, 245.7,  -140,4,0,0,0,0,0);
      InKin.IK(&theta1[5],&theta2[5],&theta3[5],141.855,  245.7,  -140,5,0,0,0,0,0);

      sprintf(msg, "LOG: theta1: %.2f, theta2: %.2f, theta3 %.2f", theta1[0], theta2[0], theta3[0]);
      push_log(msg);
      push_log((char*)"---------------");
      
      SetAngles(theta1,theta2,theta3,10,10,10);
      if(currentmillis - startUp_startTime >= 5000) 
      {
        startUp = 1;
        standBegin = 0;
      }
    }

    //Teleop demo Mode
    if(mode == 1 && startUp == 1)
    {
      stepStartFlag = 0;
      static elapsedMillis kinematicModeStartTimer;

      if(kinematicModeStartFlag == 0)
      {
        kinematicModeStartFlag = 1;
        kinematicModeStartTimer = 0;
      }

      if(kinematicModeStartTimer <= 1000)
      {
        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,1000,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,1000,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,1000,troll,tpitch,-tyaw,0);
        if(ConstrainCheck01(T_theta1,T_theta2,T_theta3))
        {
          SetAngles(T_theta1,T_theta2,T_theta3,0,0,0);
        }
      }
      else
      {
        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,0,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,0,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,0,troll,tpitch,-tyaw,0);
        if(ConstrainCheck01(T_theta1,T_theta2,T_theta3))
        {
          SetAngles(T_theta1,T_theta2,T_theta3,0,0,0);
        }
      }
    }

    //SetAngle Mode
    else if(mode == 2 && startUp == 1)
    {
      stepStartFlag = 0;
      kinematicModeStartFlag = 0;
      
      A_theta1[0] = 0;
      A_theta2[0] = 0;
      A_theta3[0] = -0;

      float px = 0, py = 0, pz = 0;
      
      FK03_inbody(px,py,pz, A_theta1[0],A_theta2[0],A_theta3[0],0);
      InKin.IK(&A_theta1[0],&A_theta2[0],&A_theta3[0],px,   py,    pz,0,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[1],A_theta2[1],A_theta3[1],1);
      InKin.IK(&A_theta1[1],&A_theta2[1],&A_theta3[1],px,   py,    pz,1,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[2],A_theta2[2],A_theta3[2],2);
      InKin.IK(&A_theta1[2],&A_theta2[2],&A_theta3[2],px,   py,    pz,2,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[3],A_theta2[3],A_theta3[3],3);
      InKin.IK(&A_theta1[3],&A_theta2[3],&A_theta3[3],px,   py,    pz,3,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[4],A_theta2[4],A_theta3[4],4);
      InKin.IK(&A_theta1[4],&A_theta2[4],&A_theta3[4],px,   py,    pz,4,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[5],A_theta2[5],A_theta3[5],5);
      InKin.IK(&A_theta1[5],&A_theta2[5],&A_theta3[5],px,   py,    pz,5,0,0,0,0,0);

      if(ConstrainCheck01(A_theta1,A_theta2,A_theta3))
      {
        SetAngles(A_theta1,A_theta2,A_theta3,20,20,20);
      }
      //mode = 2;
    }
    
    //SetNextpathPoint Mode
    else if(mode == 3 && startSetPath == 1 && startUp == 1)
    {
      kinematicModeStartFlag = 0;
      static elapsedMillis stepStartTimer = 1001;
	    if (stepStartFlag == 0 && standBegin == 1)
	    {
	      stepStartTimer = 0;
	      stepStartFlag = 1;
        standBegin = 0;
      }
      if(stepStartTimer <= 1000)
      {
        SetNextPathPoint(XPath,YPath,ZPath,TurnPath,1000);
        ConstrainCheck2(theta1,theta2,theta3);
        SetAngles(theta1,theta2,theta3,0,0,0);
        prevtime = millis();
      }
      else
      {
        curtime = millis();
        SetNextPathPoint(XPath,YPath,ZPath,TurnPath,dt);
        ConstrainCheck2(theta1,theta2,theta3);
        SetAngles(theta1,theta2,theta3,0,0,0);
      }
    }
  }


}



// ** Functions **
void SetNextPathPoint(float XP[][Pathsize*2-2],float YP[][Pathsize*2-2],float ZP[][Pathsize*2-2],float* TurnP,float d_t)
{
  static int currentPathPoint[6] = {3,9,3,9,3,9};
  static int currentPathPoint_tw[2] = {3,3};
  float x,y,z,yaww;

  if(d_t != -1000)
  {
    for(int i = 0;i<6;i++)
    {
      x = XP[i][currentPathPoint[i]];
      y = YP[i][currentPathPoint[i]];
      z = ZP[i][currentPathPoint[i]];
      yaww = TurnP[currentPathPoint_tw[i%2]];
  
      InKin.IK(&theta1[i],&theta2[i],&theta3[i],x,y,z,i,d_t,rollInput,pitchInput,yaww);
  
    }
  
    if(curtime - prevtime >= d_t)
    {
      prevtime = curtime;
  
      for(int i=0;i<2;i++)
      {
        if(currentPathPoint_tw[i] >= 11)
        {
          currentPathPoint_tw[i] = 0;
        }
        else
        {
          currentPathPoint_tw[i] = currentPathPoint_tw[i] + 1;
        }
      }
      
      for(int i=0;i<6;i++)
      {
        if(currentPathPoint[i] >= 11)
        {
          currentPathPoint[i] = 0;
        }
        else
        {
          currentPathPoint[i] = currentPathPoint[i] + 1;
        }
      }

      if(currentPathPoint[0] == 7 || currentPathPoint[1] == 7)
      {
        FeetOnFloor.data = 1;
        pub_FeetOnFloorFlag.publish(&FeetOnFloor); 
      }
    }
  }
  else
  {
    for(int i = 0;i<6;i++)
    {
      currentPathPoint_tw[i%2] = 3;
      currentPathPoint[i] = (i%2 == 0) ? 3 : 9;
    }
    prevtime = curtime;
    stepStartFlag = 0;

    if(standBegin == 0)
    {
      for(int i = 0;i<6;i++)
      {
        x = XP[i][currentPathPoint[i]];
        y = YP[i][currentPathPoint[i]];
        z = -140.0;
        yaww = TurnP[currentPathPoint_tw[i%2]];
    
        InKin.IK(&theta1[i],&theta2[i],&theta3[i],x,y,z,i,1000,rollInput,pitchInput,yaww);
      }
    }
  }
}

float AngleSmoothPrev = 150;
float AngleSmooth = 0;

void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
{
  double angles[18];
  double speeds[18];
  
  for(int leg_id = 0; leg_id<6; leg_id++)
  {
    angles[leg_id*3] = th1[leg_id]*180.0/M_PI+150.0;
    angles[leg_id*3+1] = -th2[leg_id]*180.0/M_PI+150.0;
    angles[leg_id*3+2] = -th3[leg_id]*180.0/M_PI+150.0;

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

void pubAngleFB(String &mfb0,String &mfb1,String &mfb2,String &mfb3,String &mfb4,String &mfb5,String &mfb6,String &mfb7,String &mfb8,String &mfb9,String &mfb10,String &mfb11,String &mfb12,String &mfb13,String &mfb14,String &mfb15,String &mfb16,String &mfb17)
{
  String feed = mfb0+ ","+mfb1+ ","+mfb2+ ","+mfb6+ ","+mfb7+ ","+mfb8+ ","+mfb12+ ","+mfb13+ ","+mfb14;
  motor_feedback[0].data = feed.c_str();
  pub_m_feed0.publish(&motor_feedback[0]);
  
//  motor_feedback[1].data = mfb1.c_str();
//  //pub_m_feed1.publish(&motor_feedback[1]);
//  
//  motor_feedback[2].data = mfb2.c_str();
//  //pub_m_feed2.publish(&motor_feedback[2]);
//  
//  motor_feedback[3].data = mfb3.c_str();
//  //pub_m_feed3.publish(&motor_feedback[3]);
//  
//  motor_feedback[4].data = mfb4.c_str();
//  //pub_m_feed4.publish(&motor_feedback[4]);
//  
//  motor_feedback[5].data = mfb5.c_str();
//  //pub_m_feed5.publish(&motor_feedback[5]);
//  
//  motor_feedback[6].data = mfb6.c_str();
//  //pub_m_feed6.publish(&motor_feedback[6]);
//  
//  motor_feedback[7].data = mfb7.c_str();
//  //pub_m_feed7.publish(&motor_feedback[7]);
//  
//  motor_feedback[8].data = mfb8.c_str();
//  //pub_m_feed8.publish(&motor_feedback[8]);
//  
//  motor_feedback[9].data = mfb9.c_str();
//  //pub_m_feed9.publish(&motor_feedback[9]);
//  
//  motor_feedback[10].data = mfb10.c_str();
//  //pub_m_feed10.publish(&motor_feedback[10]);
//  
//  motor_feedback[11].data = mfb11.c_str();
//  //pub_m_feed11.publish(&motor_feedback[11]);
//  
//  motor_feedback[12].data = mfb12.c_str();
//  //pub_m_feed12.publish(&motor_feedback[12]);
//  
//  motor_feedback[13].data = mfb13.c_str();
//  //pub_m_feed13.publish(&motor_feedback[13]);
//  
//  motor_feedback[14].data = mfb14.c_str();
//  //pub_m_feed14.publish(&motor_feedback[14]);
//  
//  motor_feedback[15].data = mfb15.c_str();
//  //pub_m_feed15.publish(&motor_feedback[15]);
//  
//  motor_feedback[16].data = mfb16.c_str();
//  //pub_m_feed16.publish(&motor_feedback[16]);
//  
//  motor_feedback[17].data = mfb17.c_str();
//  //pub_m_feed17.publish(&motor_feedback[17]);
  
}
