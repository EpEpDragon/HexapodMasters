# version 430

layout(local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

// TODO Make these uniforms
const int EXTENTS = 16;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 16;                       // Cells per distance unit
const int   HMAP_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

// Kernel navigation constants
const int K_SIZE = 11;
const int K_SIZE_2 = K_SIZE*K_SIZE;
const int K_SIZE_1_2 = int((K_SIZE-1)*0.5);
const float K_SIZE_INV = 1.0/K_SIZE;

layout(location = 0) uniform ivec3 sdf_index;
layout(std430, binding = 1) volatile buffer sdf { float sdf_buffer[256][256]; };
layout(std430, binding = 3) volatile buffer score { float score_buffer[256][256]; };

float kernel[K_SIZE][K_SIZE] = {{7.788007830714049 ,8.146473164114145 ,8.436648165963836 ,8.650222931107413 ,8.780954309205613 ,8.824969025845954 ,8.780954309205613 ,8.650222931107413 ,8.436648165963836 ,8.146473164114145 ,7.788007830714049 ,},
{8.146473164114145 ,8.521437889662113 ,8.824969025845954 ,9.048374180359595 ,9.185122844014574 ,9.231163463866357 ,9.185122844014574 ,9.048374180359595 ,8.824969025845954 ,8.521437889662113 ,8.146473164114145 ,},
{8.436648165963836 ,8.824969025845954 ,9.139311852712282 ,9.370674633774033 ,9.51229424500714 ,9.559974818330998 ,9.51229424500714 ,9.370674633774033 ,9.139311852712282 ,8.824969025845954 ,8.436648165963836 ,},
{8.650222931107413 ,9.048374180359595 ,9.370674633774033 ,9.607894391523232 ,9.753099120283327 ,9.801986733067553 ,9.753099120283327 ,9.607894391523232 ,9.370674633774033 ,9.048374180359595 ,8.650222931107413 ,},
{8.780954309205613 ,9.185122844014574 ,9.51229424500714 ,9.753099120283327 ,9.900498337491682 ,9.950124791926823 ,9.900498337491682 ,9.753099120283327 ,9.51229424500714 ,9.185122844014574 ,8.780954309205613 ,},
{8.824969025845954 ,9.231163463866357 ,9.559974818330998 ,9.801986733067553 ,9.950124791926823 ,10.0 ,9.950124791926823 ,9.801986733067553 ,9.559974818330998 ,9.231163463866357 ,8.824969025845954 ,},
{8.780954309205613 ,9.185122844014574 ,9.51229424500714 ,9.753099120283327 ,9.900498337491682 ,9.950124791926823 ,9.900498337491682 ,9.753099120283327 ,9.51229424500714 ,9.185122844014574 ,8.780954309205613 ,},
{8.650222931107413 ,9.048374180359595 ,9.370674633774033 ,9.607894391523232 ,9.753099120283327 ,9.801986733067553 ,9.753099120283327 ,9.607894391523232 ,9.370674633774033 ,9.048374180359595 ,8.650222931107413 ,},
{8.436648165963836 ,8.824969025845954 ,9.139311852712282 ,9.370674633774033 ,9.51229424500714 ,9.559974818330998 ,9.51229424500714 ,9.370674633774033 ,9.139311852712282 ,8.824969025845954 ,8.436648165963836 ,},
{8.146473164114145 ,8.521437889662113 ,8.824969025845954 ,9.048374180359595 ,9.185122844014574 ,9.231163463866357 ,9.185122844014574 ,9.048374180359595 ,8.824969025845954 ,8.521437889662113 ,8.146473164114145 ,},
{7.788007830714049 ,8.146473164114145 ,8.436648165963836 ,8.650222931107413 ,8.780954309205613 ,8.824969025845954 ,8.780954309205613 ,8.650222931107413 ,8.436648165963836 ,8.146473164114145 ,7.788007830714049 ,},
};

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
    return prox_score + steepness_score;
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