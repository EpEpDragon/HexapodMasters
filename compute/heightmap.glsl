# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// Uniforms
layout(location = 0) uniform ivec3 sdf_index;
layout(location = 1) uniform vec4 camera_quat;

// CPU Shared Buffers 
layout(std430, binding = 0) readonly restrict buffer image { float depth_image[90][160]; };
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[128][128]; };

const float FOV = 60;
const float FX = (90/2) / tan(FOV * 3.141 / 180 / 2);
const float FY = FX;
const float CX = (160-1) / 2.0;
const float CY = (90-1) / 2.0;
const float ZFAR = 5.4;
const float ZNEAR = 0.05;


// TODO Make these uniforms
const int EXTENTS = 16;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 8;                       // Cells per distance unit
const int SDF_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

const float PENETRATION_DEPTH = 2*DIVISIOINS;
const ivec3 ORIGIN = ivec3(SDF_EXTENTS/2);      // Origin of camera in SDF grid


// float linearize_depth(depth):
//     zlinear = (ZNEAR * ZFAR) / (ZFAR + depth * (ZNEAR - ZFAR));
//     return zlinear

// Rotate a vector p by quaternion q
vec3 rotate(vec4 q, vec3 p) 
{
  return p + 2.0*cross(q.xyz,cross(q.xyz,p)+q.w*p);
}

// Compute point in SDF grid based on depth and and rotation of camera
vec4 compute_point()
{
    int i = int(gl_GlobalInvocationID.y);
    int j = int(gl_GlobalInvocationID.x);
    float z = depth_image[i][j];
    
    // Terminate if too close to camera
    if(z < 2.5){
        return(vec4(0,0,0,2));
    }

    float clip = 0.0;
    // HACK Make this not use only z, it should clip by distance
    if(z > ZFAR-0.0005) 
    { 
        z = ZFAR;
        clip = 1.0;
    };
    float x = (j - CX) * z / FX;
    float y = (i - CY) * z / FY;
    // vec3 point = vec3(x,y,z); 
    return (vec4((rotate(camera_quat, vec3(-x,y,z)) + EXTENTS/2)*DIVISIOINS, clip));
}

void draw_to_height(vec4 point) 
{   
    if (point.w != 1.0) 
    {
        ivec2 index = ivec2(int(mod((point.x-sdf_index.x), SDF_EXTENTS)), int(mod((point.y-sdf_index.y), SDF_EXTENTS)));
        // sdf_buffer[index[0]][index[1]] = max(point.z-sdf_index.z, sdf_buffer[index[0]][index[1]]);
        sdf_buffer[index[0]][index[1]] = point.z-sdf_index.z;
    }
}

void erase_out_of_range()
{
    // if ((erase.x.start <= x && x <= erase.x.end) ||
    //     (erase.y.start <= y && y <= erase.y.end))
    // {
    //     sdf_buffer[x][y] = SDF_EXTENTS/2.0;
    // }
    if (sdf_index.x == gl_GlobalInvocationID.x || sdf_index.y == gl_GlobalInvocationID.y)
    {
        sdf_buffer[gl_GlobalInvocationID.x][gl_GlobalInvocationID.y] = float(SDF_EXTENTS)/2.0;
    }
}

void main() {
    draw_to_height(compute_point());
}