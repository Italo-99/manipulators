# Code description: Remote API Python to interact with CoppeliaSim

# FATTO DA ME PER LA TIZIA

# IMPORT LIBRARIES
# Import Coppelia libraries
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# Import python libraries
import time

# Moving function
def mover(start_pose,goal_pose,exec_time,target,sim,simIK,ikEnv,ikGroup):

    # INPUTS:
    #       start_pose: starting pose as list[3]
    #       goal_pose:  final    pose as list[3]
    #       exec_time:  duration of the movement in seconds

    # Set loop rate
    loop_rate = 0.01
    # Set discretization step
    disc = int(exec_time/loop_rate)
    step = [0.,0.,0.]
    for i in range(3):
        step[i] = (goal_pose[i]-start_pose[i])/disc
    
    # Mover loop
    next_pose = start_pose
    # Iterate over the discretization length
    for k in range(disc):
        # Update next pose
        for i in range(3):
            next_pose[i] = next_pose[i]+step[i]
        # Execute move
        sim.setObjectPosition(target, -1, next_pose)
        res, *_ = simIK.handleGroup(ikEnv,ikGroup)
        # Log unsolved IKine
        if res != simIK.result_success:
            print("Ik solver failed")
        # Wait for next iteration
        time.sleep(loop_rate)

# Close the gripper
def close_gripper(sim):

    print("Close the gripper")
    sim.setInt32Signal('RG2_open',0)
    time.sleep(2)
        
# Open the gripper
def open_gripper(sim):
    
    print("Open the gripper")
    sim.setInt32Signal('RG2_open',1)
    time.sleep(2)    

# Main working pipeline for coppelia custom simulation
def coppelia_pipeline():

    # Log python client init
    print('Python program loading')

    # Load packages
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

    # Start the simulation
    sim.loadScene('/home/italo/Downloads/italo_scene2.ttt')  # TODO: CHANGE THE PATH FILE
    sim.startSimulation()
    print('Simulation started from Python API')

    # Open the gripper as default starting state
    open_gripper(sim)

    # Declare simulation objects
    simBase = sim.getObject('/myRobot')
    simTip = sim.getObject('/myRobot/robot_tip')
    simTarget = sim.getObject('/myRobot/robot_target')

    # Create an undamped inverse kinematic chain
    ikEnv = simIK.createEnvironment()
    ikGroup_undamped = simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv, ikGroup_undamped, simIK.method_pseudo_inverse, 0, 6)
    simIK.addElementFromScene(ikEnv, ikGroup_undamped, simBase, simTip, simTarget, simIK.constraint_pose)
    # Create a damped inverse kinematic chain
    ikGroup_damped = simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv, ikGroup_damped, simIK.method_damped_least_squares, 1, 99)
    simIK.addElementFromScene(ikEnv, ikGroup_damped, simBase, simTip, simTarget, simIK.constraint_pose)
    
    # Working pipeline
    print("IK solver initialization")
    time.sleep(2)

    # Request IK solver
    res, *_ = simIK.handleGroup(ikEnv,ikGroup_undamped)
    # res, *_ = simIK.handleGroup(ikEnv,ikGroup_damped)
    if res == simIK.result_success:
        print("Kinematic chain properly initialized: IK solver activated")
    else:
        print("Kinematic chain init failed")
    
    print("Start move")

    cube    = sim.getObject('/Cuboid')
    # gripper = sim.getObject('/ROBOTIQ85')
    target  = sim.getObject('/manipSphere')    
    # sensor  = sim.getObject('/Proximity_sensor')

    # TODO: START WORKING FROM HERE, DON'T CHANGE ABOVE CODE !!!

    # Get current pose
    pose_1_3 = sim.getObjectPosition(target, -1)
    # Set final pose
    pose_2 = sim.getObjectPosition(cube, -1)
    
    # Mover function
    exec_time = 1
    mover(pose_1_3,pose_2,exec_time,target,sim,simIK,ikEnv,ikGroup_undamped)

    print("First goal reached")
    time.sleep(1)

    # Close the gripper to grab the cube
    close_gripper(sim)

    # Back to starting position
    final_pose = [0.32, 0., 0.28]
    mover(pose_2,final_pose,exec_time,target,sim,simIK,ikEnv,ikGroup_undamped)

    print("Second goal reached")
    time.sleep(1)

    # Stop the simulation
    print("Stop the simulation")
    sim.stopSimulation()

    # # Assegna al "tip" le coordinate del cubo
    # coordinate_cubo = sim.getObjectPosition(cuboHandle, -1)
    # sim.setObjectPosition(simTip, -1, coordinate_cubo)
    # sim.solveIkGroup(ikGroupHandle)

    # time.sleep(2)  # Aspetta che il braccio raggiunga la posizione

    # # Controlla il sensore di prossimità prima di chiudere il gripper
    # rilevato, _ = sim.readProximitySensor(sensorHandle)
    # if rilevato:
    #     sim.setJointTargetVelocity(gripperHandle, -0.05)  # Chiudi il gripper
    #     time.sleep(1)

    #     # Ottieni la posizione attuale del "tip"
    #     _, posizione_attuale = sim.getObjectPosition(simTip, -1)

    #     # Calcola la nuova posizione spostando il "tip" di 20 cm verso l'alto lungo l'asse Z
    #     nuova_posizione = [posizione_attuale[0], posizione_attuale[1], posizione_attuale[2] + 0.2]

    #     # Imposta la nuova posizione del "tip" per alzare il braccio
    #     sim.setObjectPosition(simTip, -1, nuova_posizione)
    #     sim.solveIkGroup(ikGroupHandle)

    #     time.sleep(2)  # Aspetta che il braccio si muova nella nuova posizione

    #     print("Cubo sollevato di 20 cm")

    # sim.stopSimulation()
  
# MAIN PYTHON EXECUTABLE FUNCTION
if __name__ == '__main__':

    coppelia_pipeline()
