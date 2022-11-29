#!/usr/bin/env python

import rospy
import time
import tf
import math

import numpy as np
import socket
import threading

import keyboard

from drone import Drone
from drone import Controller

end = False


'''
#----- A simple TCP based server program in Python using send() function -----

 

import socket

 

# Create a stream based socket(i.e, a TCP socket)

# operating on IPv4 addressing scheme

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);

 

# Bind and listen

serverSocket.bind(("192.168.1.130",6231));

serverSocket.listen();

 

# Accept connections

(clientConnected, clientAddress) = serverSocket.accept();

print("Accepted a connection request from %s:%s"%(clientAddress[0], clientAddress[1]));

while(True):

    

   

    dataFromClient = clientConnected.recv(1024)

    print(dataFromClient.decode())
'''



def start_server(ip:str, port:int):
    # Create a stream based socket(i.e, a TCP socket)

    # operating on IPv4 addressing scheme

    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    

    # Bind and listen

    serverSocket.bind((ip,port))

    serverSocket.listen()

    

    # Accept connections

    print('waiting for server accept')

    (clientConnected, clientAddress) = serverSocket.accept()

    print("Accepted a connection request from %s:%s"%(clientAddress[0], clientAddress[1]));

    return (clientConnected, clientAddress)

      


def callback(msg, drone, procedureRun):
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_rad]
    

    if procedureRun:
        drone.update_vel(pos, verbose = True, rotPosVels = True)
    
    if drone.at_setpoint(pos, admittedErrs = [0.2, 0.2, math.pi/4]):
        Drone.procedureRun = False


def thread_function():
    global end
    input()

    print('stopping and landing...')

    end = True

    #args[0].stop()
    #args[0].land()

    #time.sleep(3)

    exit(1)
    


def main():

    contParams = [
    	[0.35, 0, 0.4],
    	[0.35, 0, 0.4],
        [0.6, 0, 0.2],
    	[1, 0, 0]
    ]

    myDrone = Drone('bebop_02', 'bebop1', Controller('PID', 3, 1, contParams), None)
    
    
    #wait after creating drones to make sure all ros components are ready
    time.sleep(2)
    

    (clientConnected, clientAddress) = start_server("192.168.1.130", 5130)

    myIn = input("'t' to takeoff: ")

    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)


    myDrone.takeoff()
    print('takeoff')

    time.sleep(1)

    myDrone.set_controller_points([0, 0, 1, 0])
    
    myIn = input("'s' to start procedure after 2 seconds: ")


    time.sleep(2)


    if (myIn != 's'):
        myDrone.land()
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')

    Drone.procedureRun = True
    
    

    myThread = threading.Thread(target=thread_function)
    myThread.start()

    #try:
    
    while(Drone.procedureRun and not end):

        posUnDec = clientConnected.recv(1024)
        pos = posUnDec.decode()
        #pos = eval('np.array(' + posS + ')')
        
        

        pos = pos.split(',')
        pos = pos[:len(pos) - 1]

        pos = [float(x) for x in pos]

        posD = [pos[0], pos[1], pos[2], 0]
        

        myDrone.update_vel(posD, verbose = False, rotPosVels = True)

        #if myDrone.at_setpoint(posD, admittedErrs = [0.2, 0.2, math.pi/4]):
        #    Drone.procedureRun = False

    #except:
    #    print('force exited')
    #    myDrone.land()
    #    time.sleep(2)
    #    exit(1)


    print('procedure finished')
    myDrone.land()
    #time.sleep(2)

    print('landing')

    #myDrone.stop()
    #myDrone.land()

    time.sleep(3)
   


if __name__ == '__main__':
    print("Starting main()")
    rospy.init_node('myNode')
    main()
    print("Done!!")

