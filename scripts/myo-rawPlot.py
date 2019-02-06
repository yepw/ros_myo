#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic and drives turtlesim

import rospy, math
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import EmgArray

class Nodo(object):
    def __init__(self):
	self.myodata = np.zeros((8,1))
	self.counter = 1;
	rospy.Subscriber("/myo_raw/myo_emg", EmgArray, self.callback)

    def callback(self, msg):
	self.counter += 1
	emgArr = msg.data
	currdata = np.asarray(emgArr).reshape((8,1))
	# currdata = np.transpose(currdata)
	# print(currdata)
	self.myodata = np.concatenate((self.myodata, currdata), axis = 1)

    def Plot(self):
	fig=plt.figure()
        # fig.show()
        ax=fig.add_subplot(111)
	print(self.counter, self.myodata.shape)
        ax.plot(np.arange(self.counter),self.myodata[0],color='b',label = "Emg[0]")
        ax.plot(np.arange(self.counter),self.myodata[1],color='r',label = "Emg[1]")
        ax.plot(np.arange(self.counter),self.myodata[2],color='g',label = "Emg[2]")
        ax.plot(np.arange(self.counter),self.myodata[3],color='c',label = "Emg[3]")
        ax.plot(np.arange(self.counter),self.myodata[4],color='m',label = "Emg[4]")
        ax.plot(np.arange(self.counter),self.myodata[5],color='y',label = "Emg[5]")
        ax.plot(np.arange(self.counter),self.myodata[6],color='k',label = "Emg[6]")
        ax.plot(np.arange(self.counter),self.myodata[7],color='w',label = "Emg[7]")
	plt.legend(loc='upper left')
        plt.draw()
	fig.savefig('/home/yeping/catkin_ws/src/ros_myo/output.png') 

if __name__ == '__main__':

    rospy.init_node('myo-raw-Plot')
    #rospy.init_node('myo-raw-Plot', anonymous=True)
    
    # Publish to the turtlesim movement topic
    '''
    myodata = np.zeros((8,1))
    counter = 0;
   
    def strength(emgArr1):
	emgArr=emgArr1.data
	# Define proportional control constant:
	K = 0.005
	currdata = np.asarray(emgArr)
	myodata = np.concatenate((myodata, currdata), axis = 1)
	
    counter += 1
    rospy.Subscriber("/myo_raw/myo_emg", EmgArray, strength)
    '''
    my_node = Nodo()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    my_node.Plot()

    
