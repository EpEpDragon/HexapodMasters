#include <Eigen>
#include "MyDynamixel.h"

const float UPPER_LEG = 1;
const float UPPER_LEG_2 = UPPER_LEG*UPPER_LEG;
const float LOWER_LEG = 1;
const float LOWER_LEG_2 = LOWER_LEG*LOWER_LEG;

const float L1 = 53.17;
// const float L1z = 8;
const float L2  = 101.88;
const float L22 = L2*L2;
const float L3  = 149.16;
const float L32 = L3*L3
// const float B = 125.54;


// The position of the base servo in robot space
const Eigen::Vector3f LEG_OFFSETS[6] {
    {108.721, 62.770},
    {0.000, 125.540},
    {-108.721, 62.770},
    {-108.721, -62.770},
    {-0.000, -125.540},
    {108.721, -62.770}
};

// The inverse quaternions of the leg offset vectors
const Eigen::Quaternionf LEG_INV_QUATS[6] {
    {0.966, 0.000, 0.000, 0.259},
    {0.707, 0.000, 0.000, 0.707},
    {0.259, 0.000, 0.000, 0.966},
    {-0.259, 0.000, 0.000, 0.966},
    {-0.707, 0.000, 0.000, 0.707},
    {-0.966, 0.000, 0.000, 0.259}
};

class IK
{
    IK();
    public:
        // Set and translate targets to leg coordintate space
        void set_final_targets(const Eigen::Vector3f targets[6]);
        // Calculate the next servo angles for the leg of the provided id
        void solve_next_angles(float& theta1, float& theta2, float& theta3, uint8_t leg_id);
    private:
        // Calculate the required servo angles to achieve the given coordintate IN LEG SPACE
        void solve_ik(float& theta1, float& theta2, float& theta3, Eigen::Vector3f target);
        // Calculate leg position for given angles IN LEG SPACE
        Eigen::Vector3f solve_fk(float theta1, float theta2, float theta3);
        // Calculate the movement vector required to move to the target
        Eigen::Vector3f solve_move_vector(Eigen::Vector3f start, Eigen::Vector3f target);
        
        Eigen::Vector3f final_targets[6];
        MyDynamixel dxl;
};