#!/usr/bin/env python

import rospy
import time
import tf
import math

from drone import Drone
from drone import Controller


evaderPos = [0] * 2

pSpeed = 1
eSpeed = 0.2


def vels_to_point(currPos, ePos, speed):
    diffs = []

    for i, p in enumerate(ePos):
        diffs.append(p - currPos[i])
    
    div = speed / (sum([x**2 for x in diffs])**0.5)
    diffs = [x*div for x in diffs]

    return diffs



def persuer_func(msg, drone:Drone, procedureRun):

    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_rad]
    
    vels = vels_to_point(evaderPos)

    if procedureRun:
        drone.update_vel(pos, verbose = True, rotPosVels = True, raw_vels = vels)
    
    if drone.at_setpoint(pos, admittedErrs = [0.2, 0.2, math.pi]):
        Drone.procedureRun = False


def evader_func(msg, drone, procedureRun):
    global evaderPos

    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_rad]
    
    evaderPos = pos[:len(pos) - 1]

    if procedureRun:
        drone.update_vel(pos, verbose = True, rotPosVels = True)
    
    if drone.at_setpoint(pos, admittedErrs = [0.2, 0.2, math.pi]):
        Drone.procedureRun = False


def main():

    contParams = [
    	[0.35, 0, 0.4],
    	[0.35, 0, 0.4],
    	[1, 0, 0]
    ]

    persuer = Drone('bebop_02', 'bebop1', Controller('PID', 2, 1, contParams), persuer_func)
    evader = Drone('bebop_02', 'bebop1', Controller('PID', 2, 1, contParams), evader_func)
    
    #wait after creating drones to make sure all ros components are ready
    time.sleep(2)
    
    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    persuer.takeoff()
    evader.takeoff()
    print('takeoff')

    persuer.set_controller_points([0, 0, 0])
    evader.set_controller_points([0, 0, 0])
    
    myIn = input("'s' to start procedure: ")

    time.sleep(2)


    if (myIn != 's'):
        persuer.land()
        evader.land()
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')

    Drone.procedureRun = True
    
    
    
    while Drone.procedureRun:
        pass

    persuer.stop_movement()
    evader.stop_movement()

    print('procedure finished')


    time.sleep(3)

    print('landing')

    persuer.land()
    evader.land()

    time.sleep(3)
   


if __name__ == '__main__':
    print("Starting main()")
    rospy.init_node('myNode')
    main()

    print("Done!!")

