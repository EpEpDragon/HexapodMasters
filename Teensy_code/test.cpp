#include <Eigen>
#include <iostream>
#include <cmath>

int main()
{   
    float ang = 45 * M_PI/180;
    Eigen::Matrix3Xd in_pts {
        {1,0,0},
        {0,1,0},
        {0,0,1}
    };

    Eigen::Matrix3Xd trans {
        {1,0,0},
        {0,1,0},
        {0,0,1}
    };
    
    Eigen::Quaterniond quats {cos(ang), 0, 0, sin(ang)};
    Eigen::Matrix3Xd out_pts;
    
    out_pts = quats * (in_pts + trans);
    
    std::cout << quats << "\n";
    std::cout << out_pts << "\n";
}