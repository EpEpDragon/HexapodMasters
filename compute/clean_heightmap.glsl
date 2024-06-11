# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// TODO Make these uniforms
const int EXTENTS = 16;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 8;                       // Cells per distance unit
const int   HMAP_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

layout(location = 0) uniform ivec3 sdf_index;
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[128][128]; };
layout(std430, binding = 3) volatile buffer score { float score_buffer[128][128]; };

float calculate_score(uint x, uint y)
{
    uint ym = uint(mod(y, HMAP_EXTENTS));
    uint xm = uint(mod(x, HMAP_EXTENTS));
    uint ym1 = uint(mod(y-1, HMAP_EXTENTS));
    uint xm1 = uint(mod(x-1, HMAP_EXTENTS));
    uint yp1 = uint(mod(y+1, HMAP_EXTENTS));
    uint xp1 = uint(mod(x+1, HMAP_EXTENTS));

    float g_x = 0.5*(sdf_buffer[ym1][xm1]   - sdf_buffer[yp1][xm1]
                    +2*sdf_buffer[ym1][xm]  - 2*sdf_buffer[yp1][xm]
                    +sdf_buffer[ym1][xp1]   - sdf_buffer[yp1][xp1]);
    
    float g_y = 0.5*(+sdf_buffer[ym1][xm1]  + 2*sdf_buffer[ym][xm1]     + sdf_buffer[yp1][xm1]
                    -sdf_buffer[ym1][xp1]   - 2*sdf_buffer[ym][xp1]     - sdf_buffer[yp1][xp1]);

    float steepness_score = sqrt(g_x*g_x + g_y*g_y)*0.5;
    return steepness_score;
}

void main()
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    if  ( HMAP_EXTENTS-sdf_index.x == x ||   HMAP_EXTENTS-sdf_index.y == y)
    {
        sdf_buffer[x][y] = 0.0;
    }
    score_buffer[x][y] = calculate_score(y,x);
}