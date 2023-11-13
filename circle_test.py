#!/usr/bin/env python

import rospy
import time
import tf
import math
import csv

from threading import Thread

from nav_msgs.msg import Odometry

import numpy as np

import socket
import threading

import keyboard

from drone import Drone
from drone import Controller

end = False
lastT = time.time()

debug = False

mPos = [0, 0, 0, 0]


rospy.init_node('myNode', anonymous=True)

mPos = [0, 0]

nameMap = {'tello_01':1, 'tello_02':2}

def opti_callback(msg, drone, procedureRun):
    global mPos
    
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    

    if (nameMap[drone.name] == 1):
        mPos[0] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    else:
        mPos[1] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]

    #if Drone.procedureRun:
    #    drone.update_vel(mPos, rotPosVels=True)


myDrones = [Drone('tello_01', 'tello', None, opti_callback, ip = '192.168.1.141'), Drone('tello_02', 'tello', None, opti_callback, ip = '192.168.10.1')]




def angle_to_point(pos, point, hasZ = False, outDeg = True):
    xDiff = point[0] - pos[0]
    yDiff = point[1] - pos[1]

    ang = math.atan2(yDiff, xDiff)

    if outDeg:
        return (ang * 180/math.pi)

    else:
        return(ang)



def genCircle(rads, r, yOff):
    x = r * math.cos(rads)
    y = r * math.sin(rads) + yOff
    
    return x, y


def emergency_stop():
    input('hit enter to stop')

    Drone.procedureRun = False
    Drone.stop_and_land_drones(myDrones)

    print('emergency stopped')
    exit(1)


def main():
    global myDrones, debug

    debug = False

    time.sleep(2)    

    Drone.create_tello_swarm()


    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    Drone.takeoff_drones(myDrones)
    print('takeoff')

    time.sleep(1)
    
    myIn = input("'s' to start procedure after 1 seconds: ")


    time.sleep(1)


    if (myIn != 's'):
        Drone.stop_and_land_drones(myDrones)
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')



    radius = 0.75
    yOffset = 0
    sPerRot = 15
    myDrones[0].set_controller_points([radius, yOffset, 1, angle_to_point([radius,yOffset], [0, yOffset])], reset=True)
    myDrones[1].set_controller_points([-radius, yOffset, 1, angle_to_point([radius+math.pi,yOffset], [0, yOffset])], reset=True)
    Drone.procedureRun = True
    
    


    input('hit enter to start rotation')

    lastTime = time.time()
    rads = 0
    
    
    thread = Thread(target=emergency_stop)
    thread.start()

    cirCent = [0, yOffset]

    freq = 0.05


    while (rads < math.pi * 4 and Drone.procedureRun):
        startT = time.time()
        currTime = startT

        diff = currTime - lastTime

        rads += diff * (math.pi*2)/sPerRot

        x1, y1 = genCircle(rads, radius, yOffset)
        x2, y2 = genCircle(rads+math.pi, radius, yOffset)
        #print(x, y)

        myDrones[0].set_controller_points([x1, y1, 1, angle_to_point(mPos[0], cirCent)], reset=False)
        myDrones[1].set_controller_points([x2, y2, 1, angle_to_point(mPos[1], cirCent)], reset=False)

        myDrones[0].update_vel(mPos[0], rotPosVels=True)
        myDrones[1].update_vel(mPos[1], rotPosVels=True)

        lastTime = currTime

        while (time.time() - startT <= freq):
            time.sleep(0.0005)
            pass

        

    myDrones[0].set_controller_points([radius, yOffset, 1, angle_to_point([radius,yOffset], [0, yOffset])], reset=True)
    myDrones[1].set_controller_points([-radius, yOffset, 1, angle_to_point([radius+math.pi,yOffset], [0, yOffset])], reset=True)

    time.sleep(2)


    Drone.procedureRun = False

    #input("hit enter to stop")


    print('procedure finished')
    Drone.stop_and_land_drones(myDrones)
    #time.sleep(2)

    print('landing')

if __name__ == '__main__':
    print("Starting main()")
    main()
    print("Done!!")

