#!/usr/bin/env python

import rospy
import time
import tf
import math

import threading

from drone import Drone
from drone import Controller




def callback(msg, drone, procedureRun):
    global pos

    #if not gate:
    #    return

    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]

    

    #if procedureRun and gate and (time.time() - lastT >= freq):
    #    gate = False
    #    print(pos)
    #    drone.update_vel(pos, verbose = True, rotPosVels = True)
    #    lastT = time.time()
    #    gate = True
    
    #if drone.at_setpoint(pos, admittedErrs = [0.2, 0.2, math.pi/4]):
    #    Drone.procedureRun = False

def thread_stop():
    input('press Enter to stop')

    Drone.procedureRun = False

    print('thread exit')


def main():
    global gate

    gate = True

    contParams = [
    	[55, 0.6, 40],
    	[55, 0.6, 40],
        [60, 0.6, 40],
    	[31.847, 0.01, 0]
    ]

    myDrone = Drone('tello_01', 'tello', Controller('PID', 3, 1, contParams), callback, ip = '192.168.10.1')#ip='192.168.1.7')
    
    
    #wait after creating drones to make sure all ros components are ready
    time.sleep(2)
    
    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    myDrone.takeoff()
    print('takeoff')

    myDrone.set_controller_points([0, 0.5, 1, 0])
    
    myIn = input("'s' to start procedure: ")



    if (myIn != 's'):
        myDrone.land()
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')

    Drone.procedureRun = True
    
    freq = 0.05

    t1 = threading.Thread(target=thread_stop)
    t1.start()

    while (Drone.procedureRun):
        startT = time.time()
        print(pos)
        myDrone.update_vel(pos, verbose = True, rotPosVels = True)
        while (time.time() - startT < freq):
            pass
    


    print('procedure finished')

    print('landing')

    myDrone.stop_movement()
    myDrone.land()
    time.sleep(3)
    #myDrone.disconnect()

    #time.sleep(2)
   


if __name__ == '__main__':
    print("Starting main()")
    rospy.init_node('myNode')
    main()
    print("Done!!")

