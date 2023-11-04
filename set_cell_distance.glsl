# version 430

layout(local_size_x = 10, local_size_y = 10, local_size_z = 10) in;

// CPU Shared Buffers
layout(std430, binding = 1) buffer sdf { float sdf_buffer[120][120][120]; };

// GPU Internal Buffers
layout(std430, binding = 2) buffer surface_points { vec3 surface_points[720][1280]; };

// TODO Make these uniforms
const int EXTENTS = 30;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 4;                       // Cells per distance unit
const int SDF_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

void main()
{
    //Calculate cell position
    vec3 cell_position = (gl_GlobalInvocationID - EXTENTS/2) / DIVISIOINS;
    
    float sdf_dist = sdf_buffer[gl_GlobalInvocationID.x][gl_GlobalInvocationID.y][gl_GlobalInvocationID.z];
    
    // Initialise min distance to absolute sdf distance
    float min_dist = abs(sdf_dist);
    
    // Find mini distance to surface
    float current_dist;
    for (int i = 0; i < 720*1280; i++)
    {
        current_dist = sqrt( dot( surface_points[i] - cell_position) );
        min_dist = min( current_dist, min_dist );
    }
    sdf_buffer[gl_GlobalInvocationID.x][gl_GlobalInvocationID.y][gl_GlobalInvocationID.z] = sign(sdf_dist) * min_dist

}