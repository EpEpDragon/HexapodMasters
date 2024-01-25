#version 430

layout (location=0) in vec3 vertexPos;
layout (location=1) in vec3 vertexColor;

uniform mat4 model;
uniform mat4 projection;

out vec3 fragmentColor;
out vec3 fragPos;


void main()
{
    gl_Position = projection * model * vec4(vertexPos, 1.0);
    fragmentColor = vertexColor;
    fragPos = vertexPos + 0.5;
}