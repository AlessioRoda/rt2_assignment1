"""
/user_interface.py

/It's a simple user interface from terminal in which user can type '1' to move the robot and '0' to stop it. 
The choice of the user is sent via Command service custom message to the /state_machine node 

/author Alessio Roda

/date May 2021
"""


"""
@package rt2_assignment1
\file user_interface.py
\brief Simple user interface to start and stop the robot
\author Carmine Recchiuto, Alessio Roda
\version 1.0
\date September 2021

\details

Client to: <BR>
    /user_interface


It's the node that provides a very simple user interface to pilot the robot, so that

if user press 1 the node sends a "start" message to the user_interface server
if user press 0 the node sends a "stop" message to the user_interface server

"""



import rospy
import time
from rt2_assignment1.srv import Command

def main():
## The main fuction of the script
#
# It initializes the client to /user_interface, then checks the input genrated by th user
# if it's 1 sends a "start" message, otherwise it sends a "stop" message
#
##
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
