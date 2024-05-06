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

int main()
{   
    Eigen::Vector3d a {207, -120.000, -140};
    Eigen::Vector3d final_targets;
    final_targets = LEG_INV_QUATS[0] * (a-LEG_OFFSETS[0]);
    Eigen::Vector3d revert = (LEG_QUATS[0] * final_targets) + LEG_OFFSETS[0];

    std::cout << final_targets << "\n\n" << revert;
    // std::cout << atan(final_targets[1]/final_targets[0])*180/M_PI << "\n";
}
