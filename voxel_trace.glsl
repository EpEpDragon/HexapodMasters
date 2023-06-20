# version 430

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 0) buffer multipliers
{
    float sdf_index[3];
};

layout(std430, binding = 1) buffer numbers
{
    float sdf[];
};

void main() {
    
}