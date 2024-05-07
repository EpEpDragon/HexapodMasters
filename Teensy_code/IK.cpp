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
        this->final_targets[i] = this->robot_to_leg_space(targets[i], i);
    }
}

void IK::test()
{
//   double curr_theta1 = this->dxl->PresentPos(0);
//   double curr_theta2 = this->dxl->PresentPos(1);
//   double curr_theta3 = this->dxl->PresentPos(2);s
//   Eigen::Vector3d present_pos = this->solve_fk(curr_theta1, curr_theta2, curr_theta3);

  Eigen::Vector3d move_dir = this->solve_move_vector(this->effector_current_positions[0], this->final_targets[0]);

  char msg[50];
  sprintf(msg, "current_pos: %.2f, %.2f, %.2f\ntarget_pos: %.2f, %.2f, %.2f", effector_current_positions[0][0], effector_current_positions[0][1], effector_current_positions[0][2], this->final_targets[0][0], this->final_targets[0][1], this->final_targets[0][2]);
  this->push_log(msg);
}


void IK::solve_next_moves(double& theta1, double& theta2, double& theta3, double& dt_theta1, double& dt_theta2, double& dt_theta3, double move_speed, uint8_t leg_id)
{
    // Snap to final target is close enough
    Eigen::Vector3d delta = this->effector_current_positions[leg_id] - this->final_targets[leg_id];
    if( delta.dot(delta) < 10*10)
    {
      IK::solve_ik(theta1, theta2, theta3, dt_theta1, dt_theta2, dt_theta3, this->final_targets[leg_id], Eigen::Vector3d{move_speed,move_speed,move_speed});
      return;
    }
    // Calculate the required movement direction
    Eigen::Vector3d move_dir = IK::solve_move_vector(this->effector_current_positions[leg_id], this->final_targets[leg_id]);   
    Eigen::Vector3d immediate_target = this->final_targets[leg_id];//this->effector_current_positions[leg_id] + move_dir;

    IK::solve_ik(theta1, theta2, theta3, dt_theta1, dt_theta2, dt_theta3, immediate_target, move_dir*move_speed);
}

Eigen::Vector3d IK::solve_current_position(int leg_id)
{
    double theta1 = this->dxl->PresentPos(leg_id*3);
    double theta2 = this->dxl->PresentPos(leg_id*3 + 1);
    double theta3 = this->dxl->PresentPos(leg_id*3 + 2);

    this->effector_current_positions[leg_id] = this->solve_fk(theta1, theta2, theta3);
    return this->effector_current_positions[leg_id];
}

 Eigen::Vector3d IK::solve_move_vector(Eigen::Vector3d start, Eigen::Vector3d target)
{
    Eigen::Vector3d diff = target - start;
    return diff.normalized();
}

// Calculate inverse kinematics
void IK::solve_ik(double& theta1, double& theta2, double& theta3, double& dt_theta1, double& dt_theta2, double& dt_theta3, Eigen::Vector3d target, Eigen::Vector3d dt_target, u_int8_t leg_id)
{
    double x = target[0];
    double y = target[1];
    double z = -target[2];

    double dt_x = dt_target[0];
    double dt_y = dt_target[1];
    double dt_z = -dt_target[2];

    //----------------------- Angles -----------------------------
    theta1 = -std::atan(y/x);

    double d = sqrt(x*x + y*y);
    double dmL1 = d-L1;
    // c squared from pythagoras
    double c2 = (z*z + dmL1*dmL1);
    double c = sqrt(c2);
    // Beta from cosine rule
    double L22pL32mc2 = (L22 + L32 - c2);
    double beta = std::acos( L22pL32mc2 / (2*L2*L3) );
    theta3 = M_PI - beta;

    // Alpha from sine rule
    double alpha = std::asin( (L3 * std::sin(beta)) / c );
    theta2 = M_PI/2 - alpha - std::atan( dmL1 / z );
    
    //-------------------- Angular rates -------------------------
    x = this->effector_current_positions[leg_id][0];
    y = this->effector_current_positions[leg_id][1];
    z = -this->effector_current_positions[leg_id][2];

    double dt_d = 2*(x*dt_x + y*dt_y);
    
    double dt_c = ((-L1 + d)*dt_d + (z*dt_z)) / c;
    
    double dt_beta = (2*L2*L3*c*dt_c) / sqrt(-L22*L32*L22pL32mc2*L22pL32mc2 + 4);
    
    double dt_alpha = L3*(c*cos(beta)*dt_beta - sin(beta)*dt_c) / (sqrt(-L32*sin(beta)/c2 + 1)*c2);
    
    dt_theta1 = abs((-x*dt_y + y*dt_x) / (x*x + y*y));

    double L1md = L1 - d;
    double L1md2 = L1md*L1md;
    double z2 = z*z;
    dt_theta2 = abs(-(((L1md)*dt_z + z*dt_d)*alpha + (L1md2 + z2)*atan(L1md/z)*dt_alpha) / (L1md2 + z2));

    dt_theta3 = abs(-dt_beta);
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

Eigen::Vector3d IK::robot_to_leg_space(Eigen::Vector3d vector, int leg_id)
{
    return LEG_INV_QUATS[leg_id] * (vector - LEG_OFFSETS[leg_id]);
}

Eigen::Vector3d IK::leg_to_robot_space(Eigen::Vector3d vector, int leg_id)
{
    return (LEG_QUATS[leg_id] * vector) + LEG_OFFSETS[leg_id];
}
