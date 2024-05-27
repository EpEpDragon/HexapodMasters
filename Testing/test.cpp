#include <Eigen>
#include <iostream>
#include <cmath>

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

const Eigen::Vector3d LEG_OFFSETS[6] {
    {108.721, -62.770, 0},
    {0.000, -125.540, 0},
    {-108.721, -62.770, 0},
    {-108.721, 62.770, 0},
    {-0.000, 125.540, 0},
    {108.721, 62.770, 0},
};

double clamp(double value, double lower, double upper)
{
    return std::max(0.0, std::min(value, 1.0));
}

int main()
{   
    Eigen::Vector3d a {207, -120.000, -140};
    Eigen::Vector3d final_targets;
    final_targets = LEG_INV_QUATS[0] * (a-LEG_OFFSETS[0]);
    Eigen::Vector3d revert = (LEG_QUATS[0] * final_targets) + LEG_OFFSETS[0];

    std::cout << final_targets << "\n\n" << revert << "\n\n";
    // std::cout << atan(final_targets[1]/final_targets[0])*180/M_PI << "\n";

    // Leg dimentions
    const double L1 = 53.17;
    const double L2  = 101.88;
    const double L3  = 149.16;
    // Used in IK
    const double L22 = L2*L2;
    const double L32 = L3*L3;

    double x = 124.46;
    double y = -0.05;
    double z = 58.29;

    double dt_x = 0;
    double dt_y = 0;
    double dt_z = -50;


    //----------------------- Angles -----------------------------
 
    double d = sqrt(x*x + y*y);
    double dmL1 = d-L1;
    // c squared from pythagoras
    double c2 = (z*z + dmL1*dmL1);
    double c = sqrt(c2);
    // Beta from cosine rule
    double L22pL32mc2 = (L22 + L32 - c2);
    double beta = std::acos( clamp( L22pL32mc2 / (2*L2*L3), 0.0, 1.0 ) );


    // Alpha from sine rule
    double alpha = std::asin( clamp((L3 * std::sin(beta)) / c, 0.0, 1.0) );

    
    //-------------------- Angular rates -------------------------
    double dt_d = (x*dt_x + y*dt_y)/d;
    double dt_c = (-(L1 - d)*dt_d + (z*dt_z)) / sqrt((L1-d)*(L1-d)+z*z);
    double dt_beta = (2*L2*L3*c*dt_c) / sqrt(abs(-L22*L32*L22pL32mc2*L22pL32mc2 + 4));
    double dt_alpha = L3*(c*cos(beta)*dt_beta - sin(beta)*dt_c) / (sqrt(abs(-L32*sin(beta)/c2 + 1))*c2);
    double dt_theta1 = abs((-x*dt_y + y*dt_x) / (x*x + y*y)) * 9.54;

    double L1md = L1 - d;
    double L1md2 = L1md*L1md;
    double z2 = z*z;
    double dt_theta2 = abs(-(((L1md)*dt_z + z*dt_d)*alpha + (L1md2 + z2)*atan(L1md/z)*dt_alpha) / (L1md2 + z2))* 9.54;

    double dt_theta3 = abs(-dt_beta)* 9.54;

    std::cout << "c: " << c << "\n";
    std::cout << "dt_c: " << dt_c << "\n";
    std::cout << "d: " << d << "\n";
    std::cout << "dt_d: " << dt_d << "\n";
    std::cout << "alpha: " << alpha << "\n";
    std::cout << "dt_alpha: " << dt_alpha << "\n";
    std::cout << "beta: " << beta << "\n";
    std::cout << "dt_beta: " << dt_beta << "\n";
    std::cout << "rate: " << dt_theta1 << " " << dt_theta2 << " " << dt_theta3 << "\n\n";
    std::cout << (2*L2*L3*c*dt_c) << "\n\n";
    std::cout << -L22*L32*L22pL32mc2*L22pL32mc2 + 4 << "\n\n";
}
