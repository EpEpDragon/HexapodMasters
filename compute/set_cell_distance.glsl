# version 430

layout(local_size_x = 10, local_size_y = 10, local_size_z = 10) in;

struct Range
{
    int start;
    int end;
};

struct EraseRange 
{
    Range x;
    Range y;
    Range z;
};

// Uniforms
layout(location = 0) uniform EraseRange erase;

// CPU Shared Buffers
layout(std430, binding = 1) restrict buffer sdf { float sdf_buffer[120][120][120]; };

// GPU Internal Buffers
layout(std430, binding = 2) readonly restrict buffer points { vec4 surface_points[90][160]; };

// TODO Make these uniforms
const int EXTENTS = 15;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 8;                       // Cells per distance unit
const int SDF_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

const int DIV = 2;

// TODO Remove branching?
void erase_out_of_range(uint x, uint y, uint z)
{
    if ((erase.x.start <= x && x <= erase.x.end) ||
        (erase.y.start <= y && y <= erase.y.end) ||
        (erase.z.start <= z && z <= erase.z.end))
    {
        sdf_buffer[x][y][z] = 10000;
    }
}

void main()
{    
    // uint x = gl_GlobalInvocationID.x;
    // uint y = gl_GlobalInvocationID.y;
    // uint z = gl_GlobalInvocationID.z;

    // Calculate cell position in world space
    // vec3 cell_position = (gl_GlobalInvocationID - EXTENTS/2) / DIVISIOINS;
    
    // float sdf_dist = sdf_buffer[gl_GlobalInvocationID.x][gl_GlobalInvocationID.y][gl_GlobalInvocationID.z];
    
    // // Initialise min distance to absolute sdf distance squared (optimisation)
    // float min_dist2 = sdf_dist*sdf_dist;
    
    // // Find min distance to surface
    // float current_dist2;
    // for (int y = 0; y < 90/DIV; y++)
    // {
    //     for (int x = 0; x < 160/DIV; x++)
    //     {
    //         if (surface_points[y*DIV][x*DIV].w != 2.0)
    //         {
    //             vec3 diff = surface_points[y*DIV][x*DIV].xyz - cell_position;
    //             current_dist2 = dot( diff, diff );
    //             min_dist2 = min( current_dist2, min_dist2 );
    //         }
    //     }
    // }

    // sdf_buffer[gl_GlobalInvocationID.x][gl_GlobalInvocationID.y][gl_GlobalInvocationID.z] = sign(sdf_dist) * sqrt( min_dist2 );
    erase_out_of_range(gl_GlobalInvocationID.x,gl_GlobalInvocationID.y,gl_GlobalInvocationID.z);
}