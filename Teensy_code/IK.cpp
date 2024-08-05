#include <cmath>
#include "IK.h"

IK::IK(MyDynamixel* dxl, void (*push_log)(char*))
{
    this->dxl = dxl;
    this->push_log = push_log;    
}

void IK::set_final_targets(Eigen::Vector3d targets[6], bool is_swinging[6])
{
    for (int i=0; i<6; i++)
    {
        this->final_targets[i] = this->robot_to_leg_space(targets[i], i);
        this->is_swinging[i] = is_swinging[i];
    }
}

//void IK::test()
//{
////   double curr_theta1 = this->dxl->PresentPos(0);
////   double curr_theta2 = this->dxl->PresentPos(1);
////   double curr_theta3 = this->dxl->PresentPos(2);s
////   Eigen::Vector3d present_pos = this->solve_fk(curr_theta1, curr_theta2, curr_theta3);
//
//  Eigen::Vector3d move_dir = this->solve_move_vector(this->effector_current_positions[0], this->final_targets[0]);
//
//  char msg[50];
//  sprintf(msg, "current_pos: %.2f, %.2f, %.2f\ntarget_pos: %.2f, %.2f, %.2f", effector_current_positions[0][0], effector_current_positions[0][1], effector_current_positions[0][2], this->final_targets[0][0], this->final_targets[0][1], this->final_targets[0][2]);
//  this->push_log(msg);
//}


void IK::solve_next_moves(double& theta1, double& theta2, double& theta3, double& dt_theta1, double& dt_theta2, double& dt_theta3, double move_speed, uint8_t leg_id)
{
    // Snap to final target is close enough
//    Eigen::Vector3d delta = this->effector_current_positions[leg_id] - this->final_targets[leg_id];
//    if( delta.dot(delta) < 10*10)
//    {
//      IK::solve_ik(theta1, theta2, theta3, dt_theta1, dt_theta2, dt_theta3, this->final_targets[leg_id], Eigen::Vector3d{move_speed,move_speed,move_speed}, leg_id);
//      return;
//    }

    // Calculate the required movement direction
    Eigen::Vector3d move_dir;   
    Eigen::Vector3d immediate_target;
    if(this->is_swinging[leg_id])
    {
      move_dir = IK::solve_move_vector(this->effector_current_positions[leg_id], this->final_targets[leg_id]);
      immediate_target = this->effector_current_positions[leg_id] + move_dir;
    }
    else
    {
      move_dir = this->final_targets[leg_id] - this->effector_current_positions[leg_id];
      immediate_target = this->final_targets[leg_id];
    }

//    move_dir = this->final_targets[leg_id] - this->effector_current_positions[leg_id];
//    immediate_target = this->final_targets[leg_id];

    move_dir.normalize();

    // if (leg_id == 5)
    // {
    //   char msg[50];
    //   sprintf(msg,"leg %i move_dir: [%.2f, %.2f, %.2f]", leg_id, move_dir[0], move_dir[1], move_dir[2]);
    //   push_log(msg);  
    // }

    IK::solve_ik(theta1, theta2, theta3, dt_theta1, dt_theta2, dt_theta3, immediate_target, move_dir*move_speed, leg_id);
}


Eigen::Vector3d IK::solve_current_position(int leg_id)
{
    double theta1 = this->dxl->PresentPos(leg_id*3);
    double theta2 = this->dxl->PresentPos(leg_id*3 + 1);
    double theta3 = this->dxl->PresentPos(leg_id*3 + 2);

//    if (leg_id == 0){
//      this->theta10 = theta1;
//      this->theta20 = theta2-HIP_PITCH_OFFSET;
//      this->theta30 = theta3-KNEE_OFFSET;
//      char msg[50];
//      sprintf(msg,"current angle: %lf, %lf, %lf", theta1, theta2-HIP_PITCH_OFFSET, theta3-KNEE_OFFSET);
//      push_log(msg);
//    }

    this->effector_current_positions[leg_id] = this->solve_fk(theta1, theta2 - HIP_PITCH_OFFSET, theta3 - KNEE_OFFSET); // Offset because I cant design parts correctly
    return this->effector_current_positions[leg_id];
}


double clamp(double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}


double Ch = 0.20;
double Cs = 0;
Eigen::Vector3d IK::solve_move_vector(Eigen::Vector3d start, Eigen::Vector3d target)
{
    // Horizontal and vertical distance for vector field function
    Eigen::Vector3d delta = start-target;
    Eigen::Vector2d vec_h = Eigen::Vector2d {delta[0], delta[1]};
    
    double dist_h = sqrt(vec_h.dot(vec_h));
    double dist_v = delta[2];

    // Sigmoid to fix starting points below target
    double sig = (0.6*(dist_h - (-11.0))) / (1 + fabs(dist_h - (-11.0))) - 0.6;
    
    // Components of parabole derivative
    double a = -fabs(Ch/dist_h) - fabs(sig);
    double b = dist_v/dist_h - a*dist_h;
    
    double slope = 2*a*dist_h + b;
//    if (dist_v != 0)
//    {
//      slope -= Cs*(dist_h/fabs(dist_v));
//    }
    
    // Return move vector with vectical calculated slope and length while maintaining original horizontal direction
    Eigen::Vector3d move_vector = Eigen::Vector3d {-delta[0], -delta[1], dist_h * -slope};
    double leng = sqrt(move_vector.dot(move_vector));
    if (leng < 40)
    {
      return move_vector;
    }
    
    // Scale down if longer
    return move_vector * (40 / leng);
//     Eigen::Vector3d diff = target - start;
//     return diff.normalized();
}


// Scale rotation rates to fit into defined range, servos are jittery below certain speeds.
void IK::scale_rates(double& dt_theta1, double& dt_theta2, double& dt_theta3)
{
  double delta_min = dt_theta1 - MIN_SERVO_SPEED;
  delta_min = min(delta_min, dt_theta2 - MIN_SERVO_SPEED);
  delta_min = min(delta_min, dt_theta3 - MIN_SERVO_SPEED);
  if(delta_min < 0)
  {
    dt_theta1 = min(dt_theta1 - delta_min, MAX_SERVO_SPEED);
    dt_theta2 = min(dt_theta2 - delta_min, MAX_SERVO_SPEED);
    dt_theta3 = min(dt_theta3 - delta_min, MAX_SERVO_SPEED);
  }
}

void IK::calc_shared_vars(double& d, double& dmL1, double& c2, double& c, double& L22pL32mc2, double& beta, double& alpha, double x, double y, double z)
{
    // Horizontal distance
    d = sqrt(x*x + y*y);
    dmL1 = d-L1;
    
    // Leg triangle distance
    c2 = (z*z + dmL1*dmL1);
    c = sqrt(c2);

    // Internal knee angle
    L22pL32mc2 = (L22 + L32 - c2);
    beta = std::acos( clamp( L22pL32mc2 / (2*L2*L3), -1.0, 1.0 ) );

    // Internal hip pitch angle
    alpha = std::asin( clamp((L3 * std::sin(beta)) / c, -1.0, 1.0) );
}

// Calculate inverse kinematics
void IK::solve_ik(double& theta1, double& theta2, double& theta3, double& dt_theta1, double& dt_theta2, double& dt_theta3, Eigen::Vector3d target, Eigen::Vector3d dt_target, uint8_t leg_id)
{
    double x = target[0];
    double y = target[1];
    double z = -target[2];
//    if (leg_id == 0){
//      char msg[50];
//      sprintf(msg,"targets: %lf, %lf, %lf", x, y, -z);
//      push_log(msg);
//    }

    double dt_x = dt_target[0];
    double dt_y = dt_target[1];
    double dt_z = dt_target[2];

    //----------------------- Angles -----------------------------
    double d, dmL1, c2, c, L22pL32mc2, beta, alpha;
    IK::calc_shared_vars(d, dmL1, c2, c, L22pL32mc2, beta, alpha, x, y, z);

//    theta1 = -std::atan(y/x);
//    theta2 = M_PI/2.0 - alpha - std::atan( dmL1 / z ) + HIP_PITCH_OFFSET; // Offset because I cant design parts correctly
//    theta3 = M_PI - beta + KNEE_OFFSET; // Offset because I cant design parts correctly

    theta1 = atan2(y,x);

    double xmL1xC1 = x - L1x*cos(theta1);
    double ymL1xS1 = y - L1x*sin(theta1);
    double zmL1z = z - L1z;
    double C3 = (xmL1xC1*xmL1xC1 + ymL1xS1*ymL1xS1 + zmL1z*zmL1z - L22 - L32)/(2*L2*L3);
    double S3 = sqrt(1-C3*C3);

    theta2 = atan2(z-L1z, sqrt(xmL1xC1*xmL1xC1 + ymL1xS1*ymL1xS1)) - atan2(L3*S3, L2 + L3*C3) + HIP_PITCH_OFFSET; // Offset because I cant design parts correctly
    theta3 = atan2(S3, C3) + KNEE_OFFSET; // Offset because I cant design parts correctly
//
//    if (leg_id == 0){
//      char msg[50];
//      sprintf(msg,"values: xmLxC1: %lf, xmLxS1: %lf, zmL1z: %lf, C3: %lf", xmL1xC1, ymL1xS1, zmL1z, C3);
//      push_log(msg);
//    }

    
    //-------------------- Angular rates -------------------------
    x = this->effector_current_positions[leg_id][0];
    y = this->effector_current_positions[leg_id][1];
    z = -this->effector_current_positions[leg_id][2];
    IK::calc_shared_vars(d, dmL1, c2, c, L22pL32mc2, beta, alpha, x, y, z);

    double dt_d = (x*dt_x + y*dt_y) / d;
    double dt_c = ((-L1 + d)*dt_d + (z*dt_z)) / c;
    double dt_beta = (c*dt_c) / (L2*L3*sqrt(fabs( 4 - L22pL32mc2*L22pL32mc2 / (L22*L32) )));
    double dt_alpha = L3*(c*cos(beta)*dt_beta - sin(beta)*dt_c) / (sqrt(fabs(-L32*sin(beta)/c2 + 1))*c2);

    double L1md = L1 - d;
    double L1md2 = L1md*L1md;
    double z2 = z*z;

    dt_theta1 = fabs((-x*dt_y + y*dt_x) / (x*x + y*y)) * RAD_TO_RPM;
    dt_theta2 = fabs(-(((L1md)*dt_z + z*dt_d)*alpha + (L1md2 + z2)*atan(L1md/z)*dt_alpha) / (L1md2 + z2)) * RAD_TO_RPM;
    dt_theta3 = fabs(-dt_beta) * RAD_TO_RPM;
    scale_rates(dt_theta1, dt_theta2, dt_theta3);//
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

    double d = L1x + L2*C2 + L3*C23;
    return Eigen::Vector3d {C1*d, S1*d, -(L1z + L3*S23 + L2*S2)};
}

Eigen::Vector3d IK::robot_to_leg_space(Eigen::Vector3d vector, int leg_id)
{
    return LEG_INV_QUATS[leg_id] * (vector - LEG_OFFSETS[leg_id]);
}

Eigen::Vector3d IK::leg_to_robot_space(Eigen::Vector3d vector, int leg_id)
{
    return (LEG_QUATS[leg_id] * vector) + LEG_OFFSETS[leg_id];
}
