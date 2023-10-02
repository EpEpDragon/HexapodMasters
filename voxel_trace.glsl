# version 430

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

layout(location = 0) uniform ivec3 sdf_index;

layout(std430, binding = 0) buffer points
{
    vec3 surface_points[];
};

layout(std430, binding = 1) buffer sdf
{
    float sdf_buffer[120][120][120];
};

const int SDF_EXTENTS = 120;

void main() {
    float t = 0.0; // Total fraction to surface point
    // vec3 to = vec3(surface_points[gl_GlobalInvocationID.x * 3], surface_points[gl_GlobalInvocationID.x * 3 + 1], surface_points[gl_GlobalInvocationID.x * 3 + 2]);
    // vec3 ray = to - sdf_index;
    vec3 ray = surface_points[gl_GlobalInvocationID.x] - sdf_index;
    vec3 dXYZ = normalize(ray);

    int X = int(sdf_index.x);
    int Y = int(sdf_index.y);
    int Z = int(sdf_index.z);

    int stepX = int(sign(dXYZ.x));
    int stepY = int(sign(dXYZ.y));
    int stepZ = int(sign(dXYZ.z));

    float tMaxX = (1-mod(X, 1)) / ray.x;
    float tMaxY = (1-mod(Y, 1)) / ray.y;
    float tMaxZ = (1-mod(Z, 1)) / ray.z;
    // float tMaxX = (X*ceil(X*to.x) - X*to.x) / dXYZ.x;
    // float tMaxY = (Y*ceil(Y*to.y) - Y*to.y) / dXYZ.y;
    // float tMaxZ = (Z*ceil(Z*to.z) - Z*to.z) / dXYZ.z;

    // TODO Change this to terminate farther away
    while(t < 1.0)
    {
        if(tMaxX < tMaxY)
        {
            if(tMaxX < tMaxZ)
            {
                X = int(mod(X + stepX, SDF_EXTENTS));
                tMaxX += dXYZ.x;
                t += dXYZ.x;
            } 
            else 
            {
                Z = int(mod(Z + stepZ, SDF_EXTENTS));
                tMaxZ += dXYZ.z;
                t += dXYZ.z;
            }
        } 
        else 
        {
            if(tMaxY < tMaxZ)
            {
                Y = int(mod(Y + stepY, SDF_EXTENTS));
                tMaxY += dXYZ.y;
                t += dXYZ.y;
            }
            else
            {
                Z = int(mod(Z + stepZ, SDF_EXTENTS));
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
    }
}