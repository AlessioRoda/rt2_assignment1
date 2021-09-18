#! /usr/bin/env python

"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Node implementing an aglorhitm to move the robot
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com

This node drive the robot to the desired position received

Subscribes to:
    /odom 

Publishes to:
	/cmd_vel 

Client: 
     /set_velocity

Service:
 	/go_to_point 

 It's performed as the node that permits the robot to move: it contains all the functions to set the velocity and the orientation of the robot
 based on the target it has to reach via the ActionGoal that was sent from the /state_machine node. In order to do this, when a new target is 
 generated the robot aligns its orienation, then moves to the target with a velocity that is proportional to the velocity set with the sliders
 in the user_interface notebook interface. Once the target is reached, the node sets the target action as succeded, then waits untill a new 
 target is sent.

"""

import rospy
import actionlib
import rt2_assignment1.msg
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rt2_assignment1.srv import Velocity
from tf import transformations
import math



position_ = Point()
''' Point to store the actual position

'''

result=rt2_assignment1.msg.PositionResult()
''' Feedback to know the state of the target 

'''

lin_coef=1
''' Initialization of linear velocity obtained from the sliders (1 since it doens't change the default velocity value) 

'''
ang_coef=1
''' Initialization of angular velocity obtained from the sliders (1 since it doens't change the default velocity value) 

'''

yaw_ = 0
''' yaw angle set to 0 as default 

'''
position_ = 0
''' position of the robot initialized to 0 

'''
state_ = 0
''' state set to 0 as default 

'''
pub_ = None
''' Initialization of variable for the publisher to /cmd_vel 

'''

velocity=None
''' Initialization of variable for the client to /set_velocity 

'''

yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
''' parameters for control 

'''

dist_precision_ = 0.1
''' maximum distance from the target allowed to consider the target reached 

'''
kp_a = -3.0 
''' angular proportional constant

'''
kp_d = 0.2
''' linear proportional constant

'''
ub_a = 0.6
''' upper bound angular velocity

'''
lb_a = -0.5
''' lower bound linear velocity 

'''
ub_d = 0.6
''' upper bound linear velocity 

'''

server=None
''' action server 

'''


def clbk_odom(msg):

    """
    Callback to get the current position of the robot from an /odom message, type Odometry
    Args:
        msg: Odometry type message

"""

    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]



def change_state(state):

    """
    Function to change from a state to another in the go_to_point function
    Args:
        state(int): int variable for the new state of the FSM

"""

    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):

    """
    Function for normalizing the angle between -pi and pi.
    Args:
        angle(Float): the input angle
    Returns:
        angle(Float): the normalized angle.
"""

    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):

    """
    Function used to orient the robot respect to the target.
    Args:
        des_pos(Point): desired position to reach

"""

    global ang_coef
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    twist_msg.angular.z=twist_msg.angular.z*ang_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):

    """
    Function to call in case the robot has to go straigth once it is correctly orriented
    Set the linear and angular velocity of the robot according to the distance to the target

    Args:
        des_pos(Point): desired position to reach

"""
    global lin_coef, ang_coef
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
           twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        
        twist_msg.angular.z=twist_msg.angular.z*ang_coef
        twist_msg.linear.x=twist_msg.linear.x*lin_coef
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def fix_final_yaw(des_yaw):

    """
    Once the goal's position is acheived the function orients the robot in order to reach the goal

    Args:
        des_yaw(float): the desired yaw

"""

    global ang_coef, lin_coef
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    twist_msg.angular.z=twist_msg.angular.z*ang_coef
    twist_msg.linear.x=twist_msg.linear.x*lin_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)

      
def done():

    """
    Function to call when the target is reached or when it's cancelled, 
    it sets to zero the linear and angualar velocity of the robot in order to stop it

"""

    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)



def go_to_point(goal):

    """
    Function that gets the goal position to reach and, based on that, calls the functions to perform the motion of the robot.
    It checks if the goal has't been cancelled; in this case it sets the state_ variable to 3 in order to terminate the process to move 
    to reach a certain position.

    It also multiplies the linear and angular velocity values sent from the sliders via Velocity custom service message
    to the current linear and angular velocity, then publishes the new velocity message to perform the correct motion to reach the goal.

    Args:
        goal(Point): the goal to reach

    Returns:
         a boolean 'True' value to notify that the function has finished

"""

    global server, state_
    result=rt2_assignment1.msg.PositionResult()
    desired_position = Point()
    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta

    change_state(0)
    while True:
        
        if server.is_preempt_requested():
            state_=3

        if state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            break
    server.set_succeeded(result)
    
    return True


def sliderVelocity(vel):

    """
    Callback to get the linar and angular velocity values sent from the user_interface by Velocity custom service message.
    It saves these values in the global variables lin_coef and ang_coef.

    Args:
        vel(Velocity): a Velocity custom message

"""

    global lin_coef, ang_coef

    lin_coef=vel.linear
    ang_coef=vel.angular


def main():

    """
    The main function for this script: it initializes the publisher, the subscribers and the SimpleActionServer, in particular

    creates a publisher on topic /cmd_vel to publish the robot's velocity
    creates a subscriber on topic /odom to get the current position of the robot
    creates a service on topic /set_velocity to get the linear and angular values from the sliders
     creates an action in order to manage the FSM

"""

    global pub_, server, velocity
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    velocity = rospy.Service('/set_velocity', Velocity, sliderVelocity)
    server = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.PositionAction, execute_cb = go_to_point, auto_start=False)
    server.start()
    
    rospy.spin()


if __name__ == '__main__':
    main()
