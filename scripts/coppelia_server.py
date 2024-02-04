#!/usr/bin/env python

"""*
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

# SERVER IMPLEMENTATION TO HANDLE COPPELIA SIMULATIONS

# IMPORT LIBRARY
from manipulators.srv import CoppeliaMenu                   # Coppelia service
import rospy, rospkg                                        # ROS libraries
import time                                                 # Python libraries
from coppeliasim_zmqremoteapi_client import RemoteAPIClient #`Coppelia Remote API`

# GLOBAL VARIABLES DECLARATION

# Setup sim client Remote API
client = RemoteAPIClient()
sim    = client.require('sim')


# COPPELIA SIM CLASS
class Coppelia:

    def __init__(self):

        # Get scene params (fill here if the user doesn't pass them)
        package_name = rospy.get_param('~package_name')
        scene_name   = rospy.get_param('~scene_name')

        if (package_name == ''):
            package_name = 'manipulators'

        if (scene_name == ''):
            scene_name = ''

        # Get user scene
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        self.scene_path = package_path + "/scenes/" + scene_name

    # Coppelia simulation starting
    def start_sim(self):
        
        sim.loadScene(self.scene_path)
        sim.startSimulation()

        return True

    # Coppelia simulation stopping
    def stop_sim(self):

        sim.stopSimulation()
        time.sleep(2)
        sim.saveScene()
        time.sleep(2)
        sim.closeScene()
        
        return False


# COPPELIA MENU HANDLER
    
# Choice among Coppelia functions according to the client request
def menu_handler(req):

    coppelia = Coppelia()

    if      req.command == 0:
        coppelia.start_sim()
        return True
    elif    req.command == 1:
        coppelia.stop_sim()
        return False
    else:
        print("Wrong command sent")
        return False

# Service and node declarations
def coppelia_menu_server():

    # Init coppelia node
    rospy.init_node('coppelia_server')
    rospy.loginfo("Coppelia server python started")
    # Init coppelia service
    rospy.Service('coppelia_menu', CoppeliaMenu, menu_handler)
    rospy.loginfo("Ready to send commands to CoppeliaSim")
    # Setup a controlled rate ROS Python spinner
    spin_rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        spin_rate.sleep()
    # Closing the node
    rospy.loginfo("Closing the node")
    rospy.signal_shutdown("Shutdown requested")


# MAIN PYTHON EXECUTABLE FUNCTION
if __name__ == '__main__':

    coppelia_menu_server()
