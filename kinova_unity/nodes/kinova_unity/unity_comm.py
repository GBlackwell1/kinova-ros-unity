#!/usr/bin/env python3

#remove or add the library/libraries for ROS
import rospy
import math
import rosservice
import actionlib
#remove or add the message type
from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from sensor_msgs.msg import JointState
from kinova_msgs.msg import JointVelocity
from kinova_msgs.msg import ArmJointAnglesAction
from kinova_msgs.msg import ArmJointAnglesGoal

degs = None
robot_name = 'm1n6s300_driver'
robot_state = 0
#define function/functions to provide the required functionaI needlity



def unity_callback(msg):
    # two types of messages available, connection and movement
    # both sent as a string and need to be parsed into something usable
    if msg.data == 'Connection to Unity : TRUE':
        print("Calling service to home "+msg.data)
        #rospy.wait_for_service('/m1n6s300_driver/in/home_arm')
        #rospy.ServiceProxy('/m1n6s300_driver/in/home_arm', HomeArm)
    else: # message is movement
        global degs
        degs = msg.data.split(':')
        for i in range(0,6):
            degs[i] = float(degs[i])
        # Parse everything into degs, length 6 arr for all joints from unity
        print('User command is... ')
        print(degs, sep=",")
        # Establish connection with client (m1n6s300)
        action_address = '/'+robot_name+'/joints_action/joint_angles'
        client = actionlib.SimpleActionClient(action_address, ArmJointAnglesAction)
        print('Made it before server wait')
        client.wait_for_server()
        print('Made it after server wait')
        # Create and send goal to the robot via client
        goal = ArmJointAnglesGoal()
        # Take the goals that were parsed and send them here
        goal.angles.joint1 = degs[0]
        goal.angles.joint2 = degs[1]
        goal.angles.joint3 = degs[2]
        goal.angles.joint4 = degs[3]
        goal.angles.joint5 = degs[4]
        goal.angles.joint6 = degs[5]
        # Send the goal to the client
        client.send_goal(goal)
        print(goal)
        # Wait for response from the client
        if client.wait_for_result(rospy.Duration(20.0)):
            return client.get_result()
        else:
            print('Joint angle action timed out')
            client.cancel_all_goals()
            return None

def jointstate_callback(msg):
    global robot_state
    # Get positions of all of current joints
    robot_state = msg.position


if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('kinova_unity')
    # Subscribe to a topic using rospy.Subscriber class
    subtool=rospy.Subscriber('/'+robot_name+'/out/joint_state', JointState, jointstate_callback)
    incoming_unity=rospy.Subscriber('/unity_incoming', String, unity_callback)
    
    rate=rospy.Rate(100)

    while not rospy.is_shutdown():
         

        # stop continuously sending data 
        rate.sleep()
		
