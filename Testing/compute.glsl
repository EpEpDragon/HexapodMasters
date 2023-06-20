# version 430

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 0) buffer numbers
{
    float numbers_SSBO[]
};

void main() {
    numbers_SSBO[gl_Global_InvocationID.x] *= 2
}