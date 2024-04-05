#!/usr/bin/env python3

#remove or add the library/libraries for ROS
import rospy
import math
import rosservice
import actionlib
import os
#remove or add the message type
from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from kinova_msgs.msg import ArmJointAnglesAction
from kinova_msgs.msg import ArmJointAnglesGoal
from kinova_msgs.msg import JointAngles
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus


received_movement = None
robot_name = 'm1n6s300_driver'
robot_state = []
curr_status = 3
#define function/functions to provide the required functionaI needlity

def sendCommand(msg, relative):
    # Establish connection with client (m1n6s300)
    action_address = '/'+robot_name+'/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address, ArmJointAnglesAction)
    client.wait_for_server()
    
    # Create and send goal to the robot via client
    goal = ArmJointAnglesGoal()

    if relative:
        print('Relative move received...')
        print(msg)
        getCurrentState()
        goal.angles.joint1 = msg[0]+robot_state[0]
        goal.angles.joint2 = msg[1]+robot_state[1]
        goal.angles.joint3 = msg[2]+robot_state[2]
        goal.angles.joint4 = msg[3]+robot_state[3]
        goal.angles.joint5 = msg[4]+robot_state[4]
        goal.angles.joint6 = msg[5]+robot_state[5]
    else:
        print('Non-relative move received...')
        goal.angles.joint1 = msg[0]
        goal.angles.joint2 = msg[1]
        goal.angles.joint3 = msg[2]
        goal.angles.joint4 = msg[3]
        goal.angles.joint5 = msg[4]
        goal.angles.joint6 = msg[5]

    # Send the goal to the client and torque goal to topic
    client.send_goal(goal)
    print(goal)
    # Wait for response from the client
    if client.wait_for_result(rospy.Duration(45.0)):
        return client.get_result()
    else:
        print('Joint angle action timed out')
        client.cancel_all_goals()
        return None


def unity_callback(msg):
    # two types of messages available, connection and movement
    # both sent as a string and need to be parsed into something usable
    if msg.data == 'Connection to Unity : TRUE':
        print("Connection recieved!")
    elif msg.data[0:4] == "HOME":
        print("User wants to move robot HOME")
        home_command = msg.data[5:].split(':')
        
        # Split all the moves and parse to float
        for i in range(0,6):
            home_command[i] = float(home_command[i])
        print('Home command is...')
        print(home_command, sep=',')
        sendCommand(home_command, False)
    else: 
        # Move towards a position
        received_movement = msg.data[7:].split(':')
        for i in range(0,6):
            received_movement[i] = float(received_movement[i])
        
        # Parse everything into degs, length 6 arr for all joints from unity
        print('User command is... ')
        print(received_movement, sep=",")
        sendCommand(received_movement, True)

def getCurrentState():
    topic_address = '/'+robot_name+'/out/joint_command'
    rospy.Subscriber(topic_address, JointAngles, setCurrentState)
    # Get positions of all of current joints
    rospy.wait_for_message(topic_address, JointAngles)
    print('Current robot state received from listener')

def setCurrentState(msg):
    global robot_state
    robot_state.clear()
    currentState_arr = str(msg).split("\n")
    for index in range(0,len(currentState_arr)):
        # Do some funky python parsing to get just joint
        temp = currentState_arr[index].split(": ")
        robot_state.append(float(temp[1]))

# This is used to send robot status back to unity
# CHECK IT PIMPS: all msg codes are under actionlib_msgs/GoalStatus.msg
def outgoing_callback(msg):
    # Get GoalStatus MSG from index 1, then get the actual uint8 status code
    status_list = msg.status_list
    # get actual data to send, false = noninteractable, true = interactable
    if not status_list:
        return
    status_temp = status_list[0]
    status_text = status_temp.text
    status = status_temp.status

    if status != curr_status:
        if status == 1: # ACTIVE: Working towards the goal
            data = "false:"+status_text
        else: # SUCCEEDED: Finished movement goal
            data = "true:"+status_text
	print("Data being sent to /kinova_outgoing: "+data)
        outgoing_kinova.publish(data)
    # TODO: Remove the publishing here and the constant printing, only print when published
    outgoing_kinova.publish(str(status))
    print(str(status)+" is the current status")

if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('kinova_unity') 
    # Subscribe to incoming unity messages from rosbridge
    global outgoing_kinova
    outgoing_kinova = rospy.Publisher('kinova_outgoing', String, queue_size=10)
    incoming_unity = rospy.Subscriber('/unity_incoming', String, unity_callback)  
    robot_status = rospy.Subscriber('/'+robot_name+'/joints_action/joint_angles/status', GoalStatusArray, outgoing_callback)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # keep spinning
        rate.sleep()
		
