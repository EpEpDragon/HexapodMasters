# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// TODO Make these uniforms
const int EXTENTS = 16;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 16;                       // Cells per distance unit
const int HMAP_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

layout(location = 0) uniform ivec3 sdf_index;
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[256][256]; };

// float kernel = {{43.4820075  26.15589338 22.15755935 21.89100375 22.15755935 26.15589338 43.4820075 }
//                 {26.15589338  8.82977926  4.83144523  4.56488963  4.83144523  8.82977926 26.15589338}
//                 {22.15755935  4.83144523  0.8331112   0.5665556   0.8331112   4.83144523 22.15755935}
//                 {21.89100375  4.56488963  0.5665556   0.3         0.5665556   4.56488963 21.89100375}
//                 {22.15755935  4.83144523  0.8331112   0.5665556   0.8331112   4.83144523 22.15755935}
//                 {26.15589338  8.82977926  4.83144523  4.56488963  4.83144523  8.82977926 26.15589338}
//                 {43.4820075  26.15589338 22.15755935 21.89100375 22.15755935 26.15589338 43.4820075 }}

// float calculate_score()
// {
//     temp = kernel - abs(baseimg[wrap_slice(baseimg,0, (y-int(kernel_size/2))%baseimg.shape[0], (y+int(kernel_size/2)+1)%baseimg.shape[0])][:,wrap_slice(baseimg,1,(x-int(kernel_size/2))%baseimg.shape[0], (x+int(kernel_size/2)+1)%baseimg.shape[0])] - baseimg[y,x,0])
//     terrain_proximity_score = max(temp.min(),0)
// }

void main()
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    // if (HMAP_EXTENTS-sdf_index.x == x || HMAP_EXTENTS-sdf_index.y == y)
    // {
    //     sdf_buffer[x][y] = 0.0;
    // }
    sdf_buffer[x][y] = 0.0;
}