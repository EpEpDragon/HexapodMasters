#include <cmath>
#include "IK.h"

IK::IK(MyDynamixel* dxl, void (*push_log)(char*))
{
    this->dxl = dxl;
    this->push_log = push_log;
}

void IK::set_final_targets(Eigen::Vector3d targets[6])
{
    for (int i=0; i<6; i++)
    {
        this->final_targets[i] = LEG_INV_QUATS[i] * (targets[i] - LEG_OFFSETS[i]);
    }
}
void IK::test()
{
  double curr_theta1 = this->dxl->PresentPos(0);
  double curr_theta2 = this->dxl->PresentPos(1);
  double curr_theta3 = this->dxl->PresentPos(2);
  Eigen::Vector3d present_pos = this->solve_fk(curr_theta1, curr_theta2, curr_theta3);

  Eigen::Vector3d move_dir = this->solve_move_vector(present_pos, this->final_targets[0]);

  char msg[50];
  sprintf(msg, "current_pos: %.2f, %.2f, %.2f\ntarget_pos: %.2f, %.2f, %.2f", present_pos[0], present_pos[1], present_pos[2], this->final_targets[0][0], this->final_targets[0][1], this->final_targets[0][2]);
  this->push_log(msg);
}
void IK::solve_next_angles(double& theta1, double& theta2, double& theta3, uint8_t leg_id)
{
    // Get servo angles in leg
    double curr_theta1 = this->dxl->PresentPos(leg_id*3);
    double curr_theta2 = this->dxl->PresentPos(leg_id*3 + 1);
    double curr_theta3 = this->dxl->PresentPos(leg_id*3 + 2);
    
    // Calculate current pos through forward kinematics
    Eigen::Vector3d present_pos = this->solve_fk(curr_theta1, curr_theta2, curr_theta3);

    // Snap to final target is close enough
    Eigen::Vector3d delta = present_pos-this->final_targets[leg_id];
    if( delta.dot(delta) < 10*10)
    {
      this->solve_ik(theta1, theta2, theta3, this->final_targets[leg_id]);
      char msg[50];
      sprintf(msg, "leg: %i at target", leg_id);
      this->push_log(msg);
      return;
    }
    // Calculate the required movement direction
    Eigen::Vector3d move_dir = this->solve_move_vector(present_pos, this->final_targets[leg_id]);
    
    
    char msg[50];
    sprintf(msg, "pos: %.2f, %.2f, %.2f", move_dir[0], move_dir[1], move_dir[2]);
//    this->push_log(msg);
    
    Eigen::Vector3d immediate_target = present_pos + move_dir;

    this->solve_ik(theta1, theta2, theta3, immediate_target);
    //this->solve_ik(theta1, theta2, theta3, this->final_targets[leg_id]);
}

Eigen::Vector3d IK::solve_move_vector(Eigen::Vector3d start, Eigen::Vector3d target)
{
    Eigen::Vector3d diff = target - start;
    return diff;
}

// Calculate inverse kinematics
void IK::solve_ik(double& theta1, double& theta2, double& theta3, Eigen::Vector3d target)
{
    double x = target[0];
    double y = target[1];
    double z = -target[2];

    theta1 = -std::atan(y/x);

    double d = sqrt(x*x + y*y);
    double dmL1 = d-L1;
    // c squared from pythagoras
    double c2 = (z*z + dmL1*dmL1);
    // Beta from cosine rule
    double beta = std::acos( (L22 + L32 - c2) / (2*L2*L3) );
    theta3 = M_PI - beta;

    // Alpha from sine rule
    double alpha = std::asin( (L3 * std::sin(beta)) / sqrt(c2) );
    theta2 = M_PI/2 - alpha - std::atan( dmL1 / z );
}

// Calculate forward kinematics
Eigen::Vector3d IK::solve_fk(double theta1, double theta2, double theta3)
{
    double C1 = cos(theta1);
    double S1 = sin(theta1);
    double C2 = cos(theta2);
    double S2 = sin(theta2);
    double C23 = cos(theta2 + theta3);
    double S23 = sin(theta2 + theta3);

    double d = L1 + L2*C2 + L3*C23;
    return Eigen::Vector3d {C1*d, S1*d, -L3*S23-L2*S2};
}
