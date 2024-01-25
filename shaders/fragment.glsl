#version 430

layout(std430, binding = 1) readonly restrict buffer sdf { float sdf_buffer[120][120][120]; };

in vec3 fragmentColor;
in vec3 fragPos;

out vec4 color;

const int EXTENTS = 15;                         // Extents of SDF block, in distance units
const int DIVISIOINS = 8;                       // Cells per distance unit
const int SDF_EXTENTS = EXTENTS*DIVISIOINS;     // Extents of SDF block, in number of cells

const int MAX_STEPS = 100;
const float SURF_DIST = 0.05;

bool InSDF(vec3 p)
{
    return 0 <= p.x && p.x <= EXTENTS && 0 <= p.y && p.y <= EXTENTS && 0 <= p.z && p.z <= EXTENTS;
}

float GetDist(vec3 p) {
    float d = sdf_buffer[int(p.x*DIVISIOINS)][int(p.y*DIVISIOINS)][int(p.z*DIVISIOINS)];
    
    return d;
}

float RayMarch(vec3 ro, vec3 rd) {
	float dO=0.;
    
    for(int i=0; i<MAX_STEPS; i++) {
    	vec3 p = ro + rd*dO;
        float dS = GetDist(p);
        dO += dS;
        if(!InSDF(p) || abs(dS)<SURF_DIST) return 1000;
    }
    
    return dO;
}

vec3 GetNormal(vec3 p) {
    vec2 e = vec2(0.2, 0);
    vec3 n = GetDist(p) - 
        vec3(GetDist(p-e.xyy), GetDist(p-e.yxy),GetDist(p-e.yyx));
    
    return normalize(n);
}

void main()
{
    vec3 world_coord = fragPos*EXTENTS;
    vec3 dir = normalize(fragPos - vec3(0.5, 0.5, 0.0));

    float d = RayMarch(world_coord, -dir);
    vec3 col = vec3(0);

    // if(d!=1000) {
        // vec3 p = world_coord + dir * d;
        // vec3 n = GetNormal(p);
        // vec3 r = reflect(dir, n);

        // float dif = dot(n, normalize(vec3(1,2,3)))*.5+.5;
        // col = vec3(dif);
    // }
    
    // col = pow(col, vec3(.4545));	// gamma correction

    vec3 p = world_coord;
    float dist = sdf_buffer[0][0][0];
    col = vec3(dist);
    color = vec4(col, 1.0);
}