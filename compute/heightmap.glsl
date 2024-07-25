# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// Uniforms
layout(location = 0) uniform ivec3 sdf_index;
layout(location = 1) uniform vec4 camera_quat;
layout(location = 2) uniform int temporal_i;
layout(location = 3) uniform float height;

// CPU Shared Buffers 
layout(std430, binding = 0) readonly restrict buffer image { float depth_image[uint(90)][uint(160)]; };
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[256][256]; };

// Temporal average buffer
layout(std430, binding = 2) volatile buffer temp { float temporal_buffer[4][256][256]; };

const uint RES_Y = uint(160);
const uint RES_X = uint(90);
const float FOV = 60;
const float FX = (RES_X/2) / tan(FOV * 3.141 / 180 / 2);
const float FY = FX;
const float CX = (RES_Y-1) / 2.0;
const float CY = (RES_X-1) / 2.0;
const float ZFAR = 5.4;
const float ZNEAR = 0.05;


// TODO Make these uniforms
const int EXTENTS = 16;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 16;                       // Cells per distance unit
const int HMAP_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

const float PENETRATION_DEPTH = 2*DIVISIOINS;
const ivec3 ORIGIN = ivec3(HMAP_EXTENTS/2);      // Origin of camera in SDF grid


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
    if(z < 3){
        return(vec4(0,0,0,2));
    }

    float clip = 0.0;
    // TODO Make this not use only z, it should clip by distance? Maybe not for performance
    if(z > ZFAR-0.0005) 
    { 
        z = ZFAR;
        clip = 1.0;
    };
    float x = (j - CX) * z / FX;
    float y = (i - CY) * z / FY;
    // vec3 point = vec3(x,y,z); 
    vec4 point = vec4((rotate(camera_quat, vec3(-x,y,z)) + vec3(EXTENTS*0.05,EXTENTS*0.5,0)), clip);
    point[0] *= DIVISIOINS;
    point[1] *= DIVISIOINS;
    return point;
}

void draw_to_height() 
{   
    vec4 point = compute_point();
    if (point.w != 1.0) 
    {
        ivec2 index = ivec2(int(mod((point.x-sdf_index.x), HMAP_EXTENTS)), int(mod((point.y-sdf_index.y), HMAP_EXTENTS)));
        // sdf_buffer[index[0]][index[1]] = max(point.z-sdf_index.z, sdf_buffer[index[0]][index[1]]);

        // temporal_buffer[temporal_i][index[0]][index[1]] = point.z;
        // float z = (temporal_buffer[0][index[0]][index[1]] + 
        //            temporal_buffer[1][index[0]][index[1]] + 
        //            temporal_buffer[2][index[0]][index[1]] + 
        //            temporal_buffer[3][index[0]][index[1]]) / 4.0;

        sdf_buffer[index[0]][index[1]] = -point.z + height;
    }
}

void main() {
    draw_to_height();
}