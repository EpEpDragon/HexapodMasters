# version 430

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 0) buffer index
{
    vec3 sdf_index;
};

layout(std430, binding = 1) buffer points
{
    float sdf_points[];
};

layout(std430, binding = 2) buffer sdf
{
    float sdf[];
};

const int SDF_EXTENTS = 120;

void main() {
    bool empty = true;
    vec3 to = vec3(sdf_points[gl_GlobalInvocationID.x * 3], sdf_points[gl_GlobalInvocationID.x * 3 + 1], sdf_points[gl_GlobalInvocationID.x * 3 + 2]);
    vec3 ray = to - sdf_index;
    float t = 0;
    int X = int(sdf_index.x);
    int Y = int(sdf_index.y);
    int Z = int(sdf_index.z);
    int stepX = int(sign(ray.x));
    int stepY = int(sign(ray.y));
    int stepZ = int(sign(ray.z));

    float tMaxX = (X*ceil(X*to.x) - X*to.x) / ray.x;
    float tMaxY = (Y*ceil(Y*to.y) - Y*to.y) / ray.y;
    float tMaxZ = (Z*ceil(Z*to.z) - Z*to.z) / ray.z;
    ray = normalize(ray);

    while(X != to.x && Y != to.y && Z != to.z)
    {
        if(tMaxX < tMaxY)
        {
            if(tMaxX < tMaxZ)
            {
                X = int(mod(X + stepX, SDF_EXTENTS));
                tMaxX = tMaxX + ray.x;
            } 
            else 
            {
                Z = int(mod(Z + stepZ, SDF_EXTENTS));
                tMaxZ = tMaxZ + ray.z;
            }
        } 
        else 
        {
            if(tMaxY < tMaxZ)
            {
                Y = int(mod(Y + stepY, SDF_EXTENTS));
                tMaxY = tMaxY + ray.y;
            }
            else
            {
                Z = int(mod(Z + stepZ, SDF_EXTENTS));
                tMaxZ = tMaxZ + ray.z;
            }
        }
    }
}