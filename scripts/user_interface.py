
"""
.. module:: user_interface
    :platform: Unix
    :synopsis: Python module for the user Interface
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com

This node implements an user interface

Service:
    /user_interface


It's the node that provides a very simple user interface to pilot the robot, so that

if user press 1 the node sends a "start" message to the user_interface server
if user press 0 the node sends a "stop" message to the user_interface server

"""


import rospy
import time
from rt2_assignment1.srv import Command


def main():

    """

    It initializes the client to /user_interface, then checks the input genrated by th user
    if it's 1 sends a "start" message, otherwise it sends a "stop" message

"""

    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Robot has been stopped")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
