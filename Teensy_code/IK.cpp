#include <cmath>
#include "IK.h"

IK::IK(MyDynamixel* dxl, void (*push_log)(char*))
{
    this->dxl = dxl;
    this->push_log = push_log;
}

void IK::set_final_targets(const Eigen::Vector3f targets[6])
{
    for (int i=0; i<6; i++)
    {
        this->final_targets[i] = LEG_INV_QUATS[i] * (targets[i] - LEG_OFFSETS[i]);
    }
}
void IK::test()
{
  float curr_theta1 = this->dxl->PresentPos(0);
  float curr_theta2 = this->dxl->PresentPos(1);
  float curr_theta3 = this->dxl->PresentPos(2);
  Eigen::Vector3f present_pos = this->solve_fk(curr_theta1, curr_theta2, curr_theta3);

  Eigen::Vector3f move_dir = this->solve_move_vector(present_pos, this->final_targets[0]);

  char msg[50];
  sprintf(msg, "current_pos: %.2f, %.2f, %.2f\ntarget_pos: %.2f, %.2f, %.2f", present_pos[0], present_pos[1], present_pos[2], this->final_targets[0][0], this->final_targets[0][1], this->final_targets[0][2]);
  this->push_log(msg);
}
void IK::solve_next_angles(float& theta1, float& theta2, float& theta3, uint8_t leg_id)
{
    // Get servo angles in leg
    float curr_theta1 = this->dxl->PresentPos(leg_id*3);
    float curr_theta2 = this->dxl->PresentPos(leg_id*3 + 1);
    float curr_theta3 = this->dxl->PresentPos(leg_id*3 + 2);
    
    // Calculate current pos through forward kinematics
    Eigen::Vector3f present_pos = this->solve_fk(curr_theta1, curr_theta2, curr_theta3);
    
    // Calculate the required movement direction
    Eigen::Vector3f move_dir = this->solve_move_vector(present_pos, this->final_targets[leg_id]);
    
    char msg[50];
    sprintf(msg, "pos: %.2f, %.2f, %.2f", move_dir[0], move_dir[1], move_dir[2]);
    this->push_log(msg);
    
    Eigen::Vector3f immediate_target = present_pos + move_dir*10;

    this->solve_ik(theta1, theta2, theta3, immediate_target);
}

Eigen::Vector3f IK::solve_move_vector(Eigen::Vector3f start, Eigen::Vector3f target)
{
    Eigen::Vector3f diff = target - start;
    return diff / sqrt(diff.dot(diff));
}

// Calculate inverse kinematics
void IK::solve_ik(float& theta1, float& theta2, float& theta3, Eigen::Vector3f target)
{
    float x = target[0];
    float y = target[0];
    float z = target[0];

    theta1 = std::atan(y/x);

    float d = sqrt(x*x + y*y);
    float dmL1 = d-L1;
    // c squared from pythagoras
    float c2 = (z*z + dmL1*dmL1);
    // Beta from cosine rule
    float beta = std::acos( (L22 + L32 - c2) / (2*L2*L3) );
    theta3 = M_PI - beta;

    // Alpha from sine rule
    float alpha = std::asin( (L3 * std::sin(beta)) / sqrt(c2) );
    theta2 = M_PI/2 - alpha - std::atan(z / dmL1);
}

// Calculate forward kinematics
Eigen::Vector3f IK::solve_fk(float theta1, float theta2, float theta3)
{
    float C1 = cos(theta1);
    float S1 = sin(theta1);
    float C2 = cos(theta2);
    float S2 = sin(theta2);
    float C23 = cos(theta2 + theta3);
    float S23 = sin(theta2 + theta3);

    float d = L1 + L2*C2 + L3*C23;
    return Eigen::Vector3f {C1*d, S1*d, -L3*S23-L2*S2};
}
