#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import OpenGL.GL as GL
from OpenGL.GL.shaders import compileProgram,compileShader
import numpy as np

EXTENTS = 16                        # Extents of SDF block, in distance units
DIVISIOINS = 8                      # Cells per distance unit
HMAP_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells

VOXEL_TRACE_INVOCAIONS = 32         # NB This must match the x and y invocations specified in voxel_trace.glsl
CELL_DISTANCE_INVOCAIONS = 32       # NB This must match the x and y invocations specified in set_cell_distance.glsl

def update():
    pub = rospy.Publisher('hmap_data_out', String, queue_size=10)
    rospy.init_node('hmap', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

# initialise compute shader
def init_shader(self, n_points):
    # Compile cell trace program
    heightmap_src = open('../../../../compute/heightmap.glsl','r').readlines()
    self.heightmap_program = compileProgram(compileShader(heightmap_src, GL.GL_COMPUTE_SHADER))

    # Compile map cleaning program
    clean_heightmap_src = open('compute/clean_heightmap.glsl','r').readlines()
    self.clean_heightmap_program = compileProgram(compileShader(clean_heightmap_src, GL.GL_COMPUTE_SHADER))
    
    self.glbuffers  = GL.glGenBuffers(3)

    # Bind image buffer
    GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 0, self.glbuffers[0])
    GL.glBufferData(GL.GL_SHADER_STORAGE_BUFFER, n_points * 4, None, GL.GL_DYNAMIC_DRAW)
    
    # Bind SDF buffer
    GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 1, self.glbuffers[1])
    GL.glBufferData(GL.GL_SHADER_STORAGE_BUFFER, self.hmap_buffer.nbytes, self.hmap_buffer, GL.GL_DYNAMIC_READ)
    

    # Bind temporal buffer
    GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 2, self.glbuffers[2])
    GL.glBufferData(GL.GL_SHADER_STORAGE_BUFFER, self.hmap_buffer.nbytes * 4, None, GL.GL_DYNAMIC_READ)

    # Bind points buffer
    # glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.glbuffers[2])
    # glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4 * 4, None, GL_DYNAMIC_COPY)

def generate_heightmap(self, depth, camera_quat, cam_hmap_i):
    # Set depth image data
    GL.glBindBuffer(GL.GL_SHADER_STORAGE_BUFFER, self.glbuffers[0])        
    GL.glBufferSubData(GL.GL_SHADER_STORAGE_BUFFER, 0, depth.nbytes, depth)

    ################### Trace stage ##################
    GL.glUseProgram(self.heightmap_program)
    
    # Set Uniforms
    GL.glUniform3i(0, cam_hmap_i[0], cam_hmap_i[1], cam_hmap_i[2])                     # Camera position
    GL.glUniform4f(1, camera_quat[0], camera_quat[1], camera_quat[2], camera_quat[3])  # Camera rotation
    GL.glUniform1i(2, self.temporal_i)
    
    GL.glDispatchCompute(int((160)/VOXEL_TRACE_INVOCAIONS), int((90)/VOXEL_TRACE_INVOCAIONS), 1)
    
    # Sync
    # glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

    # ################# Distance stage ##################
    GL.glUseProgram(self.clean_heightmap_program)
    
    # Set Uniforms
    GL.glUniform3i(0, self.hmap_index[0], self.hmap_index[1], self.hmap_index[2])         # Robot position
    
    GL.glDispatchCompute(int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), 1)

    # Sync, read SDF
    GL.glMemoryBarrier(GL.GL_SHADER_STORAGE_BARRIER_BIT)
    GL.glBindBuffer(GL.GL_SHADER_STORAGE_BUFFER, self.glbuffers[1])
    self.hmap_buffer[:] = np.frombuffer(GL.glGetBufferSubData(GL.GL_SHADER_STORAGE_BUFFER, 0, self.hmap_buffer.nbytes), dtype=self.hmap_buffer.dtype).reshape(HMAP_EXTENTS,HMAP_EXTENTS)[:]

if __name__ == '__main__':
    try:
        init_shader()
        update()
    except rospy.ROSInterruptException:
        pass