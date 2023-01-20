#!/usr/bin/env python

import rospy
import time
import tf
import math

from drone import Drone
from drone import Controller



def callback(msg, drone, procedureRun):
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_rad]
    

    if procedureRun:
        drone.update_vel(pos, verbose = True, rotPosVels = True)
    
    #if drone.at_setpoint(pos, admittedErrs = [0.2, 0.2, math.pi/4]):
    #    Drone.procedureRun = False

def main():

    contParams = [
    	[0.35, 0.006, 0.4],
    	[0.35, 0.006, 0.4],
    	[1, 0, 0]
    ]

    myDrone = Drone('bebop_01', 'bebop1', Controller('PID', 2, 1, contParams), callback)
    
    
    #wait after creating drones to make sure all ros components are ready
    time.sleep(2)
    
    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    myDrone.takeoff()
    print('takeoff')

    myDrone.set_controller_points([0, 0, 0])
    
    myIn = input("'s' to start procedure: ")


    time.sleep(1)


    if (myIn != 's'):
        myDrone.land()
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')

    Drone.procedureRun = True
    
    
    
    input()


    print('procedure finished')

    print('landing')

    myDrone.stop_movement()
    myDrone.land()

    time.sleep(2)
   


if __name__ == '__main__':
    print("Starting main()")
    rospy.init_node('myNode')
    main()
    print("Done!!")

