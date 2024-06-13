# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// TODO Make these uniforms
const int EXTENTS = 16;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 16;                       // Cells per distance unit
const int   HMAP_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

// Kernel navigation constants
const int K_SIZE = 7;
const int K_SIZE_2 = K_SIZE*K_SIZE;
const int K_SIZE_1_2 = int((K_SIZE-1)*0.5);
const float K_SIZE_INV = 1.0/K_SIZE;

layout(location = 0) uniform ivec3 sdf_index;
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[256][256]; };
layout(std430, binding = 3) volatile buffer score { float score_buffer[256][256]; };

float kernel[K_SIZE][K_SIZE] = {{0.36787944117144233 ,0.4856717852477124 ,0.5737534207374327 ,0.6065306597126334 ,0.5737534207374327 ,0.4856717852477124 ,0.36787944117144233},
                      {0.4856717852477124 ,0.6411803884299546 ,0.7574651283969664 ,0.8007374029168081 ,0.7574651283969664 ,0.6411803884299546 ,0.4856717852477124},
                      {0.5737534207374327 ,0.7574651283969664 ,0.8948393168143698 ,0.9459594689067654 ,0.8948393168143698 ,0.7574651283969664 ,0.5737534207374327},
                      {0.6065306597126334 ,0.8007374029168081 ,0.9459594689067654 ,1.0                ,0.9459594689067654 ,0.8007374029168081 ,0.6065306597126334},
                      {0.5737534207374327 ,0.7574651283969664 ,0.8948393168143698 ,0.9459594689067654 ,0.8948393168143698 ,0.7574651283969664 ,0.5737534207374327},
                      {0.4856717852477124 ,0.6411803884299546 ,0.7574651283969664 ,0.8007374029168081 ,0.7574651283969664 ,0.6411803884299546 ,0.4856717852477124},
                      {0.36787944117144233 ,0.4856717852477124 ,0.5737534207374327 ,0.6065306597126334 ,0.5737534207374327 ,0.4856717852477124 ,0.36787944117144233}};

float calculate_score(uint x, uint y)
{
    // Gradient score
    uint ym = uint(mod(y, HMAP_EXTENTS));
    uint xm = uint(mod(x, HMAP_EXTENTS));
    uint ym1 = uint(mod(y-1, HMAP_EXTENTS));
    uint xm1 = uint(mod(x-1, HMAP_EXTENTS));
    uint yp1 = uint(mod(y+1, HMAP_EXTENTS));
    uint xp1 = uint(mod(x+1, HMAP_EXTENTS));

    float g_x = 0.5*(sdf_buffer[xm1][ym1]   - sdf_buffer[xp1][ym1]
                    +2*sdf_buffer[xm1][ym]  - 2*sdf_buffer[xp1][ym]
                    +sdf_buffer[xm1][yp1]   - sdf_buffer[xp1][yp1]);
    
    float g_y = 0.5*(sdf_buffer[xm1][ym1]   + 2*sdf_buffer[xm][ym1]     + sdf_buffer[xp1][ym1]
                    -sdf_buffer[xm1][yp1]   - 2*sdf_buffer[xm][yp1]     - sdf_buffer[xp1][yp1]);

    float steepness_score = sqrt(g_x*g_x + g_y*g_y);


    // Proximity score
    float prox_score = 0.0;
    // nested loops are funky here
    for(int i=0; i<K_SIZE_2; i++)
    {   
        int iroll = int(i*(K_SIZE_INV));
        int jroll = int(mod(i, K_SIZE));
        prox_score += kernel[iroll][jroll]*(sdf_buffer[x-K_SIZE_1_2+iroll][y-K_SIZE_1_2+jroll] - sdf_buffer[x][y]);
    }
    prox_score = abs(prox_score/(K_SIZE_2));
    return prox_score + steepness_score*0.5;
}


void main()
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    score_buffer[x][y] = calculate_score(x,y);
    if  ( HMAP_EXTENTS-sdf_index.x == x ||   HMAP_EXTENTS-sdf_index.y == y)
    {
        sdf_buffer[x][y] = 0.0;
    }
}