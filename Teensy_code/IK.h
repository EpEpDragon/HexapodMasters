#ifndef IK_h
#define IK_h

#include <ArduinoEigenSparse.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigen.h>
#include <ros.h>

#include "Arduino.h"
#include "MyDynamixel.h"

// Leg dimentions
const double L1 = 53.17;
const double L2  = 101.88;
const double L3  = 149.16;
// Used in IK
const double L22 = L2*L2;
const double L32 = L3*L3;

const double RAD_TO_RPM = 9.549297;

// The position of the base servo in robot space
const Eigen::Vector3d LEG_OFFSETS[6] {
    {108.721, -62.770, 0},
    {0.000, -125.540, 0},
    {-108.721, -62.770, 0},
    {-108.721, 62.770, 0},
    {-0.000, 125.540, 0},
    {108.721, 62.770, 0},
};

// The inverse quaternions of the leg offset vectors
const Eigen::Quaterniond LEG_INV_QUATS[6] {
    {0.966, 0.000, 0.000, 0.259},
    {0.707, 0.000, 0.000, 0.707},
    {0.259, 0.000, 0.000, 0.966},
    {-0.259, 0.000, 0.000, 0.966},
    {-0.707, 0.000, 0.000, 0.707},
    {-0.966, 0.000, 0.000, 0.259}
};

const Eigen::Quaterniond LEG_QUATS[6] {
    {0.966, 0.000, 0.000, -0.259},
    {0.707, 0.000, 0.000, -0.707},
    {0.259, 0.000, 0.000, -0.966},
    {-0.259, 0.000, 0.000, -0.966},
    {-0.707, 0.000, 0.000, -0.707},
    {-0.966, 0.000, 0.000, -0.259}
};

double clamp(double value, double lower, double upper);

class IK
{
    public:
        IK(MyDynamixel* dxl, void (*push_log)(char*));
//        void test/();
        // Set and translate targets to leg coordintate space
        void set_final_targets(Eigen::Vector3d targets[6]);
        // Calculate the next servo angles for the leg of the provided id
        void solve_next_moves(double& theta1, double& theta2, double& theta3, double& dt_theta1, double& dt_theta2, double& dt_theta3, double move_speed, uint8_t leg_id);
        // Calculate current position
        Eigen::Vector3d solve_current_position(int leg_id);
        Eigen::Vector3d leg_to_robot_space(Eigen::Vector3d vector, int leg_id);
    private:
        // Calculate the required servo angles to achieve the given coordintate IN LEG SPACE
        void solve_ik(double& theta1, double& theta2, double& theta3, double& dt_theta1, double& dt_theta2, double& dt_theta3, Eigen::Vector3d target, Eigen::Vector3d dt_target, uint8_t leg_id);
        // Calculate leg position for given angles IN LEG SPACE
        Eigen::Vector3d solve_fk(double theta1, double theta2, double theta3);
        // Calculate the movement vector required to move to the target
        Eigen::Vector3d solve_move_vector(Eigen::Vector3d start, Eigen::Vector3d target);
        Eigen::Vector3d robot_to_leg_space(Eigen::Vector3d vector, int leg_id);
        void calc_shared_vars(double& d, double& dmL1, double& c2, double& c, double& L22pL32mc2, double& beta, double& alpha, double x, double y, double z);
        

        Eigen::Vector3d final_targets[6];
        
//        const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
        MyDynamixel* dxl;
        void (*push_log)(char*);
        Eigen::Vector3d effector_current_positions[6];
};

#endif
