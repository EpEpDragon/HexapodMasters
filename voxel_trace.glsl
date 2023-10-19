# version 430

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

layout(location = 0) uniform ivec3 sdf_index;

layout(std430, binding = 0) buffer image
{
    float depth_image[720][1280];
};

layout(std430, binding = 1) buffer sdf
{
    float sdf_buffer[120][120][120];
};

const float FX = 623.533;
const float FY = 623.533;
const float CX = 639.5;
const float CY = 359.5;
const float ZFAR = 10.0;

const int EXTENTS = 30;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 4;                       // Cells per distance unit
const int SDF_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells
const float PENETRATION_DEPTH = 20.0;
const ivec3 ORIGIN = ivec3(SDF_EXTENTS/2);      // Origin of camera in SDF grid
// float linearize_depth(depth):
//     zlinear = (ZNEAR * ZFAR) / (ZFAR + depth * (ZNEAR - ZFAR));
//     return zlinear

vec4 compute_point() {
    int i = int(gl_GlobalInvocationID.y);
    int j = int(gl_GlobalInvocationID.x);
    float z = depth_image[i][j];
    // if(z > ZFAR - 0.0005) { z = ZFAR; }
    float clip = 0.0;
    if(z == 0.0 ) 
    { 
        z = ZFAR;
        clip = 1.0;
    };
    float x = (j - CX) * z / FX;
    float y = (i - CY) * z / FY;
    // return vec3(x,y,z);
    return (vec4((vec3(x,y,z) + EXTENTS/2)*DIVISIOINS, clip));
}

void main() {
    vec4 point = compute_point();
    bool far_clip_point = false; // Indicates point is not on surfaced but on far clipping plane
    // if (point.z == ZFAR) { far_clip_point = true; }
    // vec3 point = vec3(60.0);
    // int X = int(point.x);
    // int Y = int(point.y);
    // int Z = int(point.z);
    // sdf_buffer[X][Y][Z] = 0.0;

    float t = 0.0; // Total fraction to surface point
    // vec3 to = vec3(surface_points[gl_GlobalInvocationID.x * 3], surface_points[gl_GlobalInvocationID.x * 3 + 1], surface_points[gl_GlobalInvocationID.x * 3 + 2]);
    // vec3 ray = point - sdf_index;
    ivec3 sdf_index_test = ivec3(10.0, 0.0, 10.0);
    sdf_index_test = sdf_index;
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
    while(!past_clip_plane && penetration_depth2 < PENETRATION_DEPTH*PENETRATION_DEPTH)
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
        if(penetration_depth2 < 0)
        {
            sdf_buffer[int(mod((X-sdf_index_test.x), SDF_EXTENTS))][int(mod((Y-sdf_index_test.y), SDF_EXTENTS))][int(mod((Z-sdf_index_test.z), SDF_EXTENTS))] = 1.0;
        }
        else
        {
            // Terminate if far clip plane reached
            if (point.a == 1.0) 
            { 
                past_clip_plane = true; 
            }
            else
            {
                sdf_buffer[int(mod((X-sdf_index_test.x), SDF_EXTENTS))][int(mod((Y-sdf_index_test.y), SDF_EXTENTS))][int(mod((Z-sdf_index_test.z), SDF_EXTENTS))] = -1.0;
            }
            
        }
    }
}