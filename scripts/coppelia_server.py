#!/usr/bin/env python

"""LICENSE
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Andrea Pupa] [italo Almirante]
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. The name of the author may not be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Andrea Pupa] [Italo Almirante]
 *  Created on: [2024-01-17]
 *"""

""" SERVER IMPLEMENTATION TO HANDLE COPPELIA SIMULATIONS
This code responses with the same number of the command sent by the client
Available commands are the following
0: open defined scene
1: start simulation
2: stop simulation 
3: save scene
4: close scene
5: change the position of an object
"""

# IMPORT LIBRARY
from manipulators.srv import CoppeliaMenu                   # Coppelia service
import rospy, rospkg                                        # ROS libraries
import time, numpy as np                                    # Python libraries
from coppeliasim_zmqremoteapi_client import RemoteAPIClient #`Coppelia Remote API`
from scipy.spatial.transform import Rotation   

# COPPELIA SIM CLASS
class Coppelia:

    def __init__(self):

        # Init coppelia node
        rospy.init_node('coppelia_server')

        # Setup sim client Remote API
        self.client = RemoteAPIClient()
        self.sim    = self.client.require('sim')

        # Get scene params (fill here if the user doesn't pass them)
        package_name = rospy.get_param('~package_name')
        scene_name   = rospy.get_param('~scene_name')

        # Get starting positions of camera and end-effector
        self.connect_name   = rospy.get_param('~connect_name')
        self.camera_name    = rospy.get_param('~camera_name')
        self.gripper_name   = rospy.get_param('~gripper_name')
        self.cam_parent     = rospy.get_param('~cam_parent')
        self.cam_x_offset   = rospy.get_param('~cam_x_offset')
        self.cam_y_offset   = rospy.get_param('~cam_y_offset')
        self.cam_z_offset   = rospy.get_param('~cam_z_offset')
        self.cam_rx         = rospy.get_param('~cam_rx')
        self.cam_ry         = rospy.get_param('~cam_ry')
        self.cam_rz         = rospy.get_param('~cam_rz')
        self.ee_parent      = rospy.get_param('~ee_parent')
        self.ee_z_offset    = rospy.get_param('~ee_z_offset')
        self.ee_rz          = rospy.get_param('~ee_rz')

        # Get user scene
        if (package_name == ''):
            package_name = 'dlos_manipulation'

        if (scene_name == ''):
            scene_name = 'dlo_manipulation.ttt'

        rospack         = rospkg.RosPack()
        package_path    = rospack.get_path(package_name)
        self.scene_path = package_path + "/scenes/" + scene_name

        # Start node execution
        self.coppelia_menu_server()

    # Compute the position of a point given the rotation matrices
    def compute_world_pos(self,x,y,z,a,b,c):

        """
        Inputs: (x,y,z) are the coords according to world frame orientation
                (a,b,c) are the angles in radians of child frame orientation
                        referred to world frame orientation
        """

        # Define rotation matrices
        Rx = np.array([ [1,         0,          0],
                        [0, np.cos(a), -np.sin(a)],
                        [0, np.sin(a), np.cos(a)]])

        Ry = np.array([ [np.cos(b), 0,  np.sin(b)],
                        [0,         1,          0],
                        [-np.sin(b), 0, np.cos(b)]])

        Rz = np.array([ [np.cos(c), -np.sin(c), 0],
                        [np.sin(c),  np.cos(c), 0],
                        [        0,          0, 1]])

        # Compute composite rotation matrix
        R = np.dot(Rx, np.dot(Ry, Rz))

        # Define vector in child frame
        vector_child = np.array([x,y,z])

        # Rotate vector to world frame
        vector_world = np.dot(R, vector_child)

        return vector_world

    # Compute the position of a point given the rotation matrices
    def compute_world_rot(self,a,b,c,rx,ry,rz):

        # Define rotation matrix for the first frame
        Rx = np.array([ [1,         0,          0],
                        [0, np.cos(a), -np.sin(a)],
                        [0, np.sin(a), np.cos(a)]])

        Ry = np.array([ [np.cos(b), 0,  np.sin(b)],
                        [0,         1,          0],
                        [-np.sin(b), 0, np.cos(b)]])

        Rz = np.array([ [np.cos(c), -np.sin(c), 0],
                        [np.sin(c),  np.cos(c), 0],
                        [        0,          0, 1]])

        R1 = np.dot(Rx, np.dot(Ry, Rz))

        # Define rotation matrix for the second frame
        Rx = np.array([ [1,          0,           0],
                        [0, np.cos(rx), -np.sin(rx)],
                        [0, np.sin(rx), np.cos(rx)]])

        Ry = np.array([ [ np.cos(ry), 0, np.sin(ry)],
                        [0,           1,          0],
                        [-np.sin(ry), 0, np.cos(ry)]])

        Rz = np.array([ [np.cos(rz), -np.sin(rz), 0],
                        [np.sin(rz),  np.cos(rz), 0],
                        [         0,           0, 1]])

        R2 = np.dot(Rx, np.dot(Ry, Rz))

        # Compute total rotation matrix
        R = np.dot(R1,R2)

        ### first transform the matrix to euler angles
        r =  Rotation.from_matrix(R)
        angles = r.as_euler("zyx",degrees=False)

        return angles

    # Set camera starting position within the sim referred to connection frame
    def set_camera_pose(self):

        # By default, camera and tool0 axis are aligned as following:
        # Cam x axis -> tool0 z axis
        # Cam y axis -> tool0 x axis
        # Cam z axis -> tool0 y axis

        # Get robot connection pose
        conn_obj = self.sim.getObject(self.connect_name) 
        conn_pos = self.sim.getObjectPosition(conn_obj, -1)
        conn_rot = self.sim.getObjectOrientation(conn_obj, -1)

        # Compute camera pose
        x, y, z = self.compute_world_pos(self.cam_x_offset,
                                        self.cam_y_offset,
                                        self.cam_z_offset,
                                        conn_rot[0],
                                        conn_rot[1],
                                        conn_rot[2])

        # Set camera pose
        camera_pos = [conn_pos[0]+x,
                      conn_pos[1]+y,
                      conn_pos[2]+z]

        camera_rot = self.compute_world_rot(
                        conn_rot[0],conn_rot[1],conn_rot[2],
                        self.cam_rx,self.cam_ry,self.cam_rz)
        
        print(camera_rot.tolist())

        # camera_rot = [conn_rot[0]+self.cam_rx,
        #               conn_rot[1]+self.cam_ry,
        #               conn_rot[2]+self.cam_rz]
        
        # Update camera pose into sim
        cam_obj = self.sim.getObject(self.camera_name)
        self.sim.setObjectOrientation(cam_obj, -1, camera_rot) 
        self.sim.setObjectPosition(cam_obj, -1, camera_pos)

        rospy.sleep(1)
        return True

    # Set camera starting position within the sim referred to connection frame
    def set_ee_pose(self):

        # By default, camera and tool0 axis are aligned

        # Get robot connection pose
        conn_obj = self.sim.getObject(self.connect_name) 
        conn_pos = self.sim.getObjectPosition(conn_obj, -1)
        conn_rot = self.sim.getObjectOrientation(conn_obj, -1)

        # # Get ee pose
        # ee_pos = [conn_pos[0]+self.cam_x_offset,
        #           conn_pos[1]+self.cam_y_offset,
        #           conn_pos[2]+self.cam_z_offset]
        # ee_rot = [conn_rot[0]+self.cam_rx,
        #               conn_rot[1]+self.cam_ry,
        #               conn_rot[2]+self.cam_rz]
        
        # ee_obj = self.sim.getObject(self.gripper_name) 
        # self.sim.setObjectPosition(ee_obj, -1, ee_pos)
        # self.sim.setObjectOrientation(ee_obj, -1, ee_rot)

        rospy.sleep(1)
        return True
    
    # Load Coppelia scene
    def open_scene(self):

        self.sim.loadScene(self.scene_path)
        
        return 0

    # Coppelia simulation starting
    def start_sim(self):
        
        self.sim.startSimulation()
        rospy.sleep(1)

        return 1

    # Coppelia simulation stopping
    def stop_sim(self):

        self.sim.stopSimulation()
        rospy.sleep(2)
        
        return 2

    # Coppelia scene saving
    def save_scene(self):

        self.sim.saveScene()
        rospy.sleep(2)

        return 3
    
    # Coppelia scene closing
    def close_scene(self):

        self.sim.closeScene()
        rospy.sleep(2)

        return 4
    
    # Set given object pose in the Coppelia scene
    def set_obj_pose(self,obj_name):

        # TODO: write function handler
        return 5
    
    # Choice among Coppelia functions according to the client request
    def menu_handler(self,req): # TODO: change numbers in menu manipulator, too

        if      req.command == 0:
            self.start_sim()
            return True
        elif    req.command == 1:
            self.stop_sim()
            return False
        else:
            print("Wrong command sent")
            return False

    # Service and node declarations
    def coppelia_menu_server(self):

        rospy.loginfo("Coppelia server python started")
        # Init coppelia service
        rospy.Service('coppelia_menu', CoppeliaMenu, self.menu_handler)
        rospy.loginfo("Ready to send commands to CoppeliaSim")
        # Setup a controlled rate ROS Python spinner
        # spin_rate = rospy.Rate(0.5)
        # while not rospy.is_shutdown():
        #     spin_rate.sleep(spin_rate)

        # Test pipeline
        # print("OPEN SCENE")
        # self.open_scene()
        print("SET CAMERA POSE")
        self.set_camera_pose()
        # print("SET EE POSE")
        # self.set_ee_pose()
        # rospy.sleep(5)
        # print("START SIM")
        # self.start_sim()
        # rospy.sleep(2)
        # print("STOP SIM")
        # self.stop_sim()
        # rospy.sleep(2)
        # print("SAVE SCENE")
        # self.save_scene()
        # rospy.sleep(2)
        # print("CLOSE SCENE")
        # self.close_scene()

        # Closing the node
        rospy.loginfo("Closing the node")
        rospy.signal_shutdown("Shutdown requested")

# MAIN PYTHON EXECUTABLE FUNCTION
if __name__ == '__main__':

    Coppelia()
