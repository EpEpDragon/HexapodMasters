# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// Uniforms
layout(location = 0) uniform ivec3 sdf_index;
layout(location = 1) uniform vec4 camera_quat;

// CPU Shared Buffers 
layout(std430, binding = 0) readonly restrict buffer image { float depth_image[90][160]; };
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[120][120][120]; };

// GPU Internal Buffers
layout(std430, binding = 2) buffer points { vec4 surface_points[90][160]; };

const float FOV = 60;
const float FX = (90/2) / tan(FOV * 3.141 / 180 / 2);
const float FY = FX;
const float CX = (160-1) / 2.0;
const float CY = (90-1) / 2.0;
const float ZFAR = 4.7;
const float ZNEAR = 0.05;


// TODO Make these uniforms
const int EXTENTS = 15;                         // Extents of SDF block, in distance units
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
void trace(vec4 point) 
{
    float t = 0.0; // Total fraction to surface point
    // vec3 to = vec3(surface_points[gl_GlobalInvocationID.x * 3], surface_points[gl_GlobalInvocationID.x * 3 + 1], surface_points[gl_GlobalInvocationID.x * 3 + 2]);
    // vec3 ray = point - sdf_index;
    // ivec3 sdf_index_test = ivec3(10.0, 0.0, 10.0);
    // sdf_index_test = sdf_index;
    vec3 ray = point.xyz - ORIGIN;
    vec3 dXYZ = 1/normalize(ray);
    float surface_depht2 = dot(ray,ray);
    // vec3 dXYZ = vec3(1/0.936, -1/0.351 , 0.0);

    // int X = int(sdf_index_test.x);
    // int Y = int(sdf_index_test.y);
    // int Z = int(sdf_index_test.z);
    int X = ORIGIN.x;
    int Y = ORIGIN.y;
    int Z = ORIGIN.z;
    sdf_buffer[X][Y][Z] = 0.0;

    int stepX = int(sign(dXYZ.x));
    int stepY = int(sign(dXYZ.y));
    int stepZ = int(sign(dXYZ.z));
    dXYZ = abs(dXYZ);

    float tMaxX = (1-mod(X, 1)) / dXYZ.x;
    float tMaxY = (1-mod(Y, 1)) / dXYZ.y;
    float tMaxZ = (1-mod(Z, 1)) / dXYZ.z;
    // float tMaxX = (X*ceil(X*to.x) - X*to.x) / dXYZ.x;
    // float tMaxY = (Y*ceil(Y*to.y) - Y*to.y) / dXYZ.y;
    // float tMaxZ = (Z*ceil(Z*to.z) - Z*to.z) / dXYZ.z;

    // TODO Change this to terminate farther away
    
    bool past_clip_plane = false;
    float penetration_depth2 = 0.0;
    // while(abs(point.x - X) >= 2 || abs(point.y - Y) >= 2 || abs(point.z - Z) >= 2)
    while(penetration_depth2 < PENETRATION_DEPTH*PENETRATION_DEPTH)
    {
        if(tMaxX < tMaxY)
        {
            if(tMaxX < tMaxZ)
            {
                X = int(mod(X + stepX, SDF_EXTENTS));
                // X = X + stepX;
                tMaxX += dXYZ.x;
                t += dXYZ.x;
            } 
            else 
            {
                Z = int(mod(Z + stepZ, SDF_EXTENTS));
                // Z = Z + stepZ;
                tMaxZ += dXYZ.z;
                t += dXYZ.z;
            }
        } 
        else 
        {
            if(tMaxY < tMaxZ)
            {
                Y = int(mod(Y + stepY, SDF_EXTENTS));
                // Y = Y + stepY;
                tMaxY += dXYZ.y;
                t += dXYZ.y;
            }
            else
            {
                Z = int(mod(Z + stepZ, SDF_EXTENTS));
                // Z = Z + stepZ;
                tMaxZ += dXYZ.z;
                t += dXYZ.z;
            }
        }
        // Set occupancy
        // if (t < 1.0) {
        //     sdf_buffer[X][Y][Z] = 1.0;
        // } else {
        //     sdf_buffer[X][Y][Z] = -1.0;
        // }
        
        penetration_depth2 = dot(ORIGIN - vec3(X,Y,Z), ORIGIN - vec3(X,Y,Z)) - surface_depht2;
        ivec3 trace_index = ivec3(int(mod((X-sdf_index.x), SDF_EXTENTS)), int(mod((Y-sdf_index.y), SDF_EXTENTS)), int(mod((Z-sdf_index.z), SDF_EXTENTS)));
        if(penetration_depth2 < 0)
        {
            sdf_buffer[trace_index.x][trace_index.y][trace_index.z] = abs(sdf_buffer[trace_index.x][trace_index.y][trace_index.z]);
        }
        else
        {
            // Terminate if far clip plane reached
            if (point.w == 1.0) 
            { 
                return;
            }
            else
            {
                sdf_buffer[trace_index.x][trace_index.y][trace_index.z] = -abs(sdf_buffer[trace_index.x][trace_index.y][trace_index.z]);
            }
            
        }
    }
}
void main() {
    vec4 point = compute_point();
    surface_points[gl_GlobalInvocationID.y][gl_GlobalInvocationID.x] = point;
    // Trace if point is not too close to camera
    if (point.w != 2.0) {
        
        trace(point);
    }
}