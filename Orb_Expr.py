#!/usr/bin/env python

import rospy
import time
import tf
import math
import csv

import sys

from nav_msgs.msg import Odometry

import numpy as np
from threading import Thread

import keyboard

from drone import Drone
from drone import Controller


class OrbDrone(Drone):

    orbPoint = [0, 0]
    pointMass = 1
    distMult = 1


    def __init__(self, name: str, type: str, controller: Controller, callback, xVel = 0, yVel = 0, mass = 1, bounds=None):
        if controller.numPosAxis != 2:
            print("Class OrbDrone only for two dimensions", file=sys.stderr)
            sys.exit(1)

        super().__init__(name, type, controller, callback, bounds)
        self.xVel = xVel
        self.yVel = yVel
        self.xAcc = 0
        self.yAcc = 0
        self.mass = mass

        self.lastTime = time.time()

    def update_vel(self, pos: list, rotPosVels: bool = True, verbose=False):
        return super().update_vel(pos, rotPosVels, verbose=verbose, raw_vels=[self.xVel, self.yVel, 0])

    def apply_acc(self):
        curr = time.time()
        self.xVel += self.xAcc * (curr - self.lastTime)
        self.yVel += self.yAcc * (curr - self.lastTime)

        self.lastTime = curr

    def change_acc(self, forces:list):
        totX = 0
        totY = 0

        for f in forces:
            totX += f[0]
            totY += f[1]

        self.xAcc = totX
        self.yAcc = totY

    def acc_to_point(self, pos):
        diff = [self.orbPoint[0] - pos[0], self.orbPoint[1] - pos[1]]

        dist = math.sqrt(diff[0]**2 + diff[1]**2) * self.distMult

        f = (self.pointMass * self.mass) / (dist**2)

        const = f / dist

        return [d*const for d in diff]

    def follow_orbit(self, pos, applF = [0, 0], verbose = False):
        self.change_acc([applF, self.acc_to_point(pos)])
        #self.apply_acc()
        self.update_vel(pos, verbose=verbose) 

        if verbose:
            print("----------*-----------")
            print(f"xAcc:{self.xAcc} -- yAcc:{self.yAcc}")
            print(f"xVel:{self.xVel} -- yVel:{self.yVel}")

    
    def reset_time(self):
        self.lastTime = time.time()





d1Pos = [0.001, 0.001, 0.001]
d2Pos = [0, 0, 0]

rospy.init_node('myNode', anonymous=True)


def drone_1_callback(msg, drone, procedureRun):
    global d1Pos
    
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)

    d1Pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]


def drone_2_callback(msg, drone, procedureRun):
    global d2Pos
    
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    d2Pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]

    

def emergency_stop():
    input()

    Drone.procedureRun = False




contParams = [
    	[0.35, 0, 0.4],
    	[0.35, 0, 0.4],
    	[1, 0, 0]
    ]


d1 = OrbDrone('bebop_03', 'bebop1', Controller('PID', 2, 1, contParams), drone_1_callback, xVel=0, yVel=-0.5, mass=1)
#d2 = OrbDrone('bebop_01', 'bebop1', Controller('PID', 2, 1, contParams), drone_2_callback, xVel=0, yVel=0, mass=1)
myDrones = [d1]#, d2]
timePerStep = 0.005

OrbDrone.orbPoint = [0, 0]
OrbDrone.pointMass = 0.05
OrbDrone.distMult = 0.5


def main():
    global d1
    
    time.sleep(2)

    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)

    Drone.takeoff_drones(myDrones)

    #d1.takeoff()

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

    

    Drone.procedureRun = True

    thread = Thread(target=emergency_stop)
    thread.start()


    forces = [[0, 0], [0, 0]]

    d1.reset_time()
    while (Drone.procedureRun):
        d1.follow_orbit(d1Pos, verbose=True)

        time.sleep(timePerStep)    
    


    print('procedure finished')
    Drone.stop_and_land_drones(myDrones)

    print('landing')

    time.sleep(2)


    '''
    # write to csv file then break
    csvfile = open('3D_DroneDataCtrl2.csv','w',encoding="utf-8")
    dronewriter = csv.writer(csvfile, quotechar = '\'', delimiter=',',quoting = csv.QUOTE_NONE,escapechar = ' ')
    # write the header
    dronewriter.writerow(["Timestamp,el_X,el_Y,el_Z,el_Vel,mocap_X,mocap_Y,mocap_Z,mocap_Yaw"])
    # write the data
    for i in range(len(timeStamps)):
        dronewriter.writerow(f"{timeStamps[i]},{elVars[i][0]},{elVars[i][1]},{elVars[i][2]},{elVars[i][3]},{mocapPos[i][0]},{mocapPos[i][1]},{mocapPos[i][2]},{mocapPos[i][3]}")
    '''


if __name__ == '__main__':
    print("Starting main()")
    main()
    print("Done!!")

