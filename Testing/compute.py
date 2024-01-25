from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram,compileShader
import numpy as np
import glfw
import threading

# Create a OpenGL invisibe window, window is requred to create OpenGL Context idk why...
#-------------
DISPLAY_WIDTH = 900
DISPLAY_HEIGHT = 900
glfw.init()
glfw.window_hint(glfw.VISIBLE, False)
window = glfw.create_window(DISPLAY_WIDTH, DISPLAY_HEIGHT, "hidden window", None, None)
glfw.make_context_current(window)
#-------------

# Compute shader code
compute_source = """
# version 430

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

layout(location = 1) uniform float multiplier;

layout(std430, binding = 0) buffer multipliers
{
    float multipliers_SSBO[];
};

layout(std430, binding = 1) buffer numbers
{
    float numbers_SSBO[];
};

void main() {
    numbers_SSBO[gl_GlobalInvocationID.x] *= multiplier;
}
"""

# Data to initialise buffer with
multipliers = np.array([2,3,4], dtype=np.float32)
numbers = np.array([1,2,3], dtype=np.float32)

# Set up buffers
#--------------------------------
# Create 2 buffers
buffer = glGenBuffers(2)

# Bind 1st buffer to binding specified in shader code
glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, buffer[0])

# Initialise 1st buffer data and usage type
glBufferData(GL_SHADER_STORAGE_BUFFER, multipliers.nbytes, multipliers, GL_DYNAMIC_READ)

# Bind 2nd buffer to binding specified in shader code
glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, buffer[1])

# Initialise 2nd buffer data and usage type
glBufferData(GL_SHADER_STORAGE_BUFFER, numbers.nbytes, numbers, GL_DYNAMIC_READ)
#--------------------------------

# Create compute program from shader code
compute_program = compileProgram(compileShader(compute_source, GL_COMPUTE_SHADER))

# Select program to use
glUseProgram(compute_program)
glUniform1f(1,2.0)

# Dispatch current program as a compute shader, with 3 workgroups cuz 3 floats in array 
glDispatchCompute(3,1,1)

# Memory barrier to wait for compute shader to execute
glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

# Read bytes back, this function should actually take a pointer as a final argument
# to store the return bytes into, but python doesn't work like that so it just returns the bytes...
bytes = glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numbers.nbytes)

# Convert bytes to float32 and print6
print(np.frombuffer(bytes, dtype=np.float32))
print(glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT,0))
# while True:
#     if input() == 'q':
#         # multipliers *= 2
#         # glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer[0])
#         # glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, multipliers.nbytes, multipliers)
        
#         glDispatchCompute(3,1,1)
#         glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

#         glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer[1])
#         bytes = glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numbers.nbytes)
#         print(np.frombuffer(bytes, dtype=np.float32))