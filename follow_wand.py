#!/usr/bin/env python

import rospy
import time
import tf
import math

from drone import Drone
from drone import Controller


#returns radians of the angle to point to the point
#pos and point are in format [xPos, yPos]
def angle_to_point(pos, point, hasZ = False, radRot = math.pi/2, outDeg = True):
    xDiff = point[0] - pos[0]
    yDiff = point[1] - pos[1]

    ang = math.atan2(yDiff, xDiff)

    if outDeg:
        return (ang * 180/math.pi)

    else:
        return(ang)


def point_from_point(basePos, distAway = 1, zPos = None):

    uVec = [math.cos(basePos[-1]), math.sin(basePos[-1])]

    if zPos != None:
        return([basePos[0] + (uVec[0] * distAway), basePos[1] + (uVec[1] * distAway), zPos, (basePos[-1]- math.pi)* 180/math.pi])

    else:
        return([basePos[0] + (uVec[0] * distAway), basePos[1] + (uVec[1] * distAway), basePos[2], (basePos[-1]- math.pi)* 180/math.pi])


def drone_callback(msg, drone, procedureRun):
    #global newSetpoint


    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    
    
    #print(f"x  : {newSetpoint[0]}")
    #print(f"y  : {newSetpoint[1]}")
    #print(f"yaw: {newSetpoint[-1]}")
    #print(f"Nyaw:{pos[-1] * 180/math.pi}")

    if procedureRun:

        drone.set_controller_points(newSetpoint, reset = False)

        #print(drone.controller.get_points())

        drone.update_vel(pos, verbose = False, rotPosVels = True)


def wand_callback(msg, drone, procedureRun):

    global newSetpoint

    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    
    newSetpoint = point_from_point(pos, distAway=1)

    #print(f"x  : {newSetpoint[0]}")
    #print(f"y  : {newSetpoint[1]}")
    #print(f"yaw: {newSetpoint[-1] * 180/math.pi}")

        
    
    



def main():


    contParams = [
    	[1.2, 0.006, 0.35],
    	[1.2, 0.006, 0.35],
        [0.6, 0.006, 0.4],
    	[1.3, 0, 0]
    ]

    contParams = [
    	[105, 0.6, 40],
    	[105, 0.6, 40],
        [100, 0.6, 10],
    	[60, 0, 0]
    ]

    myDrone = Drone('tello_01', 'tello', Controller('PID', 3, 1, contParams), drone_callback, ip='192.168.1.141', bounds=[(-1.0, 1.0),(-1.5, 1.9),(0.8, 2.0)])
    Drone('wand_01', 'wand', None, wand_callback)
    
    
    #wait after creating drones to make sure all ros components are ready
    time.sleep(2)

    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    myDrone.takeoff()

    print('takeoff')

    time.sleep(2)

    
    myIn = input("'s' to start procedure after 2 seconds: ")


    if (myIn != 's'):
        myDrone.land()

        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)




    time.sleep(2)
    
    

    print('procedure start')


    #prevTime = time.time()
    Drone.procedureRun = True
    
    
    input("hit enter to stop and land:")
    

    Drone.procedureRun = False
    myDrone.stop_movement()

    print('procedure finished')


    print('landing')


    myDrone.land()


    time.sleep(3)
   


if __name__ == '__main__':
    print("Starting main()")
    
    #print(angle_to_point([0, -1], [0, 0]))

    rospy.init_node('myNode')
    main()

    print("Done!!")

