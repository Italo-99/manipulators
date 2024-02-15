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
0: start simulation
1: stop simulation 
2: save scene
3: change the position of an object
"""

# IMPORT LIBRARY
from manipulators.srv import CoppeliaMenu                   # Coppelia service
import rospy, rospkg                                        # ROS libraries
import time, numpy as np                                    # Python libraries
from coppeliasim_zmqremoteapi_client import RemoteAPIClient #`Coppelia API`
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

    # Compute the 3x3 rotation matrix from fixed xyz triad angles (a,b,c)
    def rot_mat(self,a,b,c):

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
        return np.dot(Rx, np.dot(Ry, Rz))

    # Compute the position of a point referred to a frame oriented as
    # the triad euler angles (a,b,c)
    def compute_world_pos(self,x,y,z,a,b,c):

        """
        Inputs: (x,y,z) are the coords according to world frame orientation
                (a,b,c) are the angles in radians of child frame orientation
                        referred to world frame orientation
        """

        # Define xyz rotation matrix frame-world
        R = self.rot_mat(a,b,c)

        # Define vector in child frame
        vector_child = np.array([x,y,z])

        # Rotate vector to world frame
        vector_world = np.dot(R, vector_child)

        return vector_world

    # Compute the orientation of a frame given the rotation matrices
    # referred to its parent frame (rx,ry,rz) and parent-world (a,b,c)
    def compute_world_rot(self,a,b,c,rx,ry,rz):

        # Define rotation matrix for the first frame
        R1 = self.rot_mat(a,b,c)

        # Define rotation matrix for the second frame
        R2 = self.rot_mat(rx,ry,rz)

        # Compute total rotation matrix
        R = np.dot(R1,R2)

        # Get euler angles from matrix
        r =  Rotation.from_matrix(R)
        angles = r.as_euler("XYZ")

        return angles.tolist()

     # Set camera starting position within the sim referred to connection frame
    
    # Set object position given its pose list [x,y,z,alpha,beta,gamma]
    # referred to a parent frame
    def set_obj_pose(self,obj_name,obj_pose,parent_name):

        # Set world as default parent
        if parent_name == '':
            parent_obj= self.sim.handle_world
        else:
            # Get parent object
            parent_obj = self.sim.getObject(parent_name)        
            
        # Get child object
        obj = self.sim.getObject(obj_name)

        # Update obj position into sim
        self.sim.setObjectPosition(obj,obj_pose[:3],parent_obj)
        
        # Update obj orientation into sim
        obj_rot = self.sim.yawPitchRollToAlphaBetaGamma(
                        obj_pose[3],obj_pose[4],obj_pose[5])
        self.sim.setObjectOrientation(obj,obj_rot,parent_obj)

        return True    

    # Set camera starting position within the sim referred to connection frame
    def set_camera_pose(self):

        # By default, camera and tool0 axis are aligned as following:
        # Cam x axis -> tool0 z axis
        # Cam y axis -> tool0 x axis
        # Cam z axis -> tool0 y axis

        return self.set_obj_pose(self.camera_name,
               [self.cam_x_offset,self.cam_y_offset,self.cam_z_offset,
                self.cam_rx,      self.cam_ry,      self.cam_rz     ],
                self.connect_name)

    # Set camera starting position within the sim referred to connection frame
    def set_ee_pose(self):

        # By default, camera and tool0 axis are aligned

        return self.set_obj_pose(self.gripper_name,
                                [0,0,self.ee_z_offset,0,0,self.ee_rz],
                                self.connect_name)
    
    # Load Coppelia scene
    def open_scene(self):

        self.sim.loadScene(self.scene_path)
        
        return 0

    # Coppelia simulation starting
    def start_sim(self):
        
        self.sim.startSimulation()
        rospy.sleep(1)
        rospy.loginfo("Starting simulation")

        return 0

    # Coppelia simulation stopping
    def stop_sim(self):

        self.sim.stopSimulation()
        rospy.loginfo("Stopping simulation")
        rospy.sleep(1)
        
        return 1

    # Coppelia scene saving
    def save_scene(self):

        rospy.loginfo("Saving scene")
        self.sim.saveScene(self.scene_path)
        rospy.sleep(1)

        return 2
    
    # Coppelia scene closing
    def close_scene(self):

        self.sim.closeScene()
        rospy.sleep(1)

        return 4
    
    # Change cable position during simulation
    def change_cable_pose(self):

        return 3

    # Choice among Coppelia functions according to the client request
    def menu_handler(self,req):

        if      req.command == 0:
            return self.start_sim()
        elif    req.command == 1:
            return self.stop_sim()
        elif    req.command == 2:
            return self.save_scene()
        elif    req.command == 3:
            return self.change_cable_pose()
        else:
            rospy.logwarn("Wrong command sent")
            return -1

    # Service and node declarations
    def coppelia_menu_server(self):

        rospy.loginfo("Coppelia server python started")
        # Init coppelia service
        rospy.Service('coppelia_menu', CoppeliaMenu, self.menu_handler)
        rospy.loginfo("Ready to send commands to CoppeliaSim")

        # Setup starting scene (CoppeliaSim must be already open)
        rospy.loginfo("Set scene")
        self.open_scene()
        rospy.sleep(1)
        rospy.loginfo("Set camera pose")
        self.set_camera_pose()
        rospy.loginfo("Set gripper pose")
        self.set_ee_pose()
        rospy.sleep(1)
        rospy.loginfo("Scene started")

        # Setup a controlled rate ROS Python spinner
        spin_rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            spin_rate.sleep()

        # Closing the scene
        self.close_scene()
        # Closing the node
        rospy.loginfo("Closing Coppelia Server node")
        rospy.signal_shutdown("Shutdown requested")

# MAIN PYTHON EXECUTABLE FUNCTION
if __name__ == '__main__':

    Coppelia()
