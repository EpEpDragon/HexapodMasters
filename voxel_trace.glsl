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

const int EXTENTS = 30;                        // Extents of SDF block, in distance units
const int DIVISIOINS = 4;                      // Cells per distance unit
const int SDF_EXTENTS = EXTENTS*DIVISIOINS;    // Extents of SDF block, in number of cells

// float linearize_depth(depth):
//     zlinear = (ZNEAR * ZFAR) / (ZFAR + depth * (ZNEAR - ZFAR));
//     return zlinear

vec3 compute_point() {
    int i = int(gl_GlobalInvocationID.y);
    int j = int(gl_GlobalInvocationID.x);
    float z = depth_image[i][j];
    float x = (j - CX) * z / FX;
    float y = (i - CY) * z / FY;
    // return vec3(x,y,z);
    return ((vec3(x,y,z)*1.5 + EXTENTS/2)*DIVISIOINS);
}

void main() {
    vec3 point = compute_point();
    // vec3 point = vec3(60.0);
    int X = int(point.x);
    int Y = int(point.y);
    int Z = int(point.z);
    sdf_buffer[X][Y][Z] = 0.0;

    float t = 0.0; // Total fraction to surface point
    // vec3 to = vec3(surface_points[gl_GlobalInvocationID.x * 3], surface_points[gl_GlobalInvocationID.x * 3 + 1], surface_points[gl_GlobalInvocationID.x * 3 + 2]);
    // vec3 ray = to - sdf_index;
    ivec3 sdf_index_test = ivec3(60.0, 60.0, 60.0);
    // vec3 ray = point - sdf_index_test;
    vec3 dXYZ = 1/normalize(point - sdf_index_test);
    // vec3 dXYZ = vec3(1/0.936, -1/0.351 , 0.0);

    X = int(sdf_index_test.x);
    Y = int(sdf_index_test.y);
    Z = int(sdf_index_test.z);
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
    int i = 0;
    while(abs(point.x - X) >= 1 || abs(point.y - Y) >= 1 || abs(point.z - Z) >= 1)
    // while(i<10)
    {
        if(tMaxX < tMaxY)
        {
            if(tMaxX < tMaxZ)
            {
                // X = int(mod(X + stepX, SDF_EXTENTS));
                X = X + stepX;
                tMaxX += dXYZ.x;
                t += dXYZ.x;
            } 
            else 
            {
                // Z = int(mod(Z + stepZ, SDF_EXTENTS));
                Z = Z + stepZ;
                tMaxZ += dXYZ.z;
                t += dXYZ.z;
            }
        } 
        else 
        {
            if(tMaxY < tMaxZ)
            {
                // Y = int(mod(Y + stepY, SDF_EXTENTS));
                Y = Y + stepY;
                tMaxY += dXYZ.y;
                t += dXYZ.y;
            }
            else
            {
                // Z = int(mod(Z + stepZ, SDF_EXTENTS));
                Z = Z + stepZ;
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
        sdf_buffer[X][Y][Z] = 0.0;
        // i += 1;
        
    }
}