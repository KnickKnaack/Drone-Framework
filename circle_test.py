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


def opti_callback(msg, drone, procedureRun):
    global mPos
    
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    mPos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]

    if Drone.procedureRun:
        drone.update_vel(mPos, rotPosVels=True)



myDrone = Drone('bebop_03', 'bebop1', None, opti_callback)


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
    myDrone.stop_movement
    myDrone.land

    print('emergency stopped')
    exit(1)


def main():
    global myDrone, debug

    debug = False

    time.sleep(2)    



    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    myDrone.takeoff()
    print('takeoff')

    time.sleep(1)
    
    myIn = input("'s' to start procedure after 1 seconds: ")


    time.sleep(1)


    if (myIn != 's'):
        myDrone.stop_movement()
        myDrone.land()
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')



    radius = 0.75
    yOffset = 0
    sPerRot = 15
    myDrone.set_controller_points([radius, yOffset, 1, angle_to_point([radius,yOffset], [0, yOffset])], reset=True)
    Drone.procedureRun = True
    
    


    input('hit enter to start rotation')

    lastTime = time.time()
    rads = 0
    
    
    thread = Thread(target=emergency_stop)
    thread.start()

    cirCent = [0, yOffset]


    while (rads < math.pi * 4 and Drone.procedureRun):
        currTime = time.time()

        diff = currTime - lastTime

        rads += diff * (math.pi*2)/sPerRot

        x, y = genCircle(rads, radius, yOffset)
        #print(x, y)

        myDrone.set_controller_points([x, y, 1, angle_to_point(mPos, cirCent)], reset=False)

        lastTime = currTime
        

    myDrone.set_controller_points([radius, yOffset, 1, angle_to_point([radius,yOffset], [0, yOffset])], reset=True)

    time.sleep(2)


    Drone.procedureRun = False

    #input("hit enter to stop")


    print('procedure finished')
    myDrone.stop_movement()
    time.sleep(1)
    myDrone.land()
    #time.sleep(2)

    print('landing')

if __name__ == '__main__':
    print("Starting main()")
    main()
    print("Done!!")

