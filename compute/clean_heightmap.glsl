# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// TODO Make these uniforms
const int EXTENTS = 16;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 8;                       // Cells per distance unit
const int SDF_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

layout(location = 0) uniform ivec3 sdf_index;
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[128][128]; };

void main()
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    if (SDF_EXTENTS-sdf_index.x == x || SDF_EXTENTS-sdf_index.y == y)
    {
        sdf_buffer[x][y] = SDF_EXTENTS/2.0;
    }
}