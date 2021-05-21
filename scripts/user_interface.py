"""
/user_interface.py

/It's a simple user interface from terminal in which user can type '1' to move the robot and '0' to stop it. 
The choice of the user is sent via Command service custom message to the /state_machine node 

/author Alessio Roda

/date May 2021
"""

import rospy
import time
from rt2_assignment1.srv import Command

def main():
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
