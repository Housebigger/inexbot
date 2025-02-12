#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommand
from gazebo_msgs.srv import *

def apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        apply_body_wrench(body_name, reference_frame, reference_point, wrench, start_time, duration)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def clear_body_wrench_client(body_name):
        rospy.wait_for_service('gazebo/clear_body_wrenches')
        try:
            clear_body_wrench = rospy.ServiceProxy('gazebo/clear_body_wrenches', BodyRequest)
            clear_body_wrench(body_name)
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)

class MoveItBigmamaDemo:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)

        
        rospy.init_node('bigmama_rolling0312', anonymous=True)
 
        
        arm = moveit_commander.MoveGroupCommander('arm_bigmama')
        
        
        hand = moveit_commander.MoveGroupCommander('hand_bigmama')
        
        
        arm.set_goal_joint_tolerance(0.001)
        hand.set_goal_joint_tolerance(0.01)

        
        
        raw_input("Press Enter to Start***")

        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = 0) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -80), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0.0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 500, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        #hand.set_named_target('handopen')
        #hand.go()
        #rospy.sleep(1)
        #hand.stop()
        raw_input("Dummy force applied, Press Enter to Continue******")

        arm.set_named_target('yewendun')
        arm.go()
        rospy.sleep(0)                 
        arm.stop()
        
        arm.set_named_target('StartState')
        arm.go()
        rospy.sleep(0)                 
        arm.stop()

        raw_input("Reached StartState, Press Enter to initiate gripper, Continue...")
    
        hand.set_named_target('handclose')
        hand.go()
        rospy.sleep(1)
        hand.stop()

        clear_body_wrench_client(body_name)

        #Starting rollover motion
        body_name = 'robot::Link_4'
        reference_frame = 'robot::Link_4'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = 0) 
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -10), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 50, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

       #body_name = 'robot::Link_innerfinger'
       #reference_frame = 'robot::Link_innerfinger'
       #reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = 0) 
       #wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
       #start_time = rospy.Time(secs = 0, nsecs = 0)
       #duration = rospy.Duration(secs = 50, nsecs = 0)
       #apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
       #body_name = 'robot::Link_exterfinger'
       #reference_frame = 'robot::Link_exterfinger'
       #reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = 0) 
       #wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
       #start_time = rospy.Time(secs = 0, nsecs = 0)
       #duration = rospy.Duration(secs = 50, nsecs = 0)
       #apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        raw_input("Link_4 force applied, Leg swithed, Press Enter to Continue......")
        #
        #joint_positions = [-0.523599, -2.094396, 0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0, -2.530729, 0]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.2617995, -2.617995, -0.2617995]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.523599, -2.530729, -0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)
        

        joint_positions = [-0.785399, -2.356196, -0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [1.570797, -2.356196, -0.785399]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions = [2.356196, -1.570797, -0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [2.356196, -1.308998, -0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [2.094396, -1.047198, -0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions =  [2.094396, -0.785399, -0.2617995]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [1.832597, -0.2617995, -0.2617995]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions =  [1.832597, -0.2617995, 0]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions =  [1.570797, 0, 0, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [1.308998, 0.2617995, 0]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [1.308998, 0.523599, 0.2617995]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions =  [1.047198, 0.785399, 0.2617995]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions =  [1.047198, 1.047198, 0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.785399, 1.308998, 0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.785399, 1.570797, 0.785399]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions = [0.785399, 2.356196, 0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [0.523599, 2.617995, 0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.523599, 2.617995, 0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.2617995, 2.356196, -0.2617995] 
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions = [0.523599, 2.094396, -0.523599, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        arm.stop()

        raw_input("Press Enter to Release Link_4 & End_Effector, Continue>>>")
        body_name = 'robot::Link_4'
        clear_body_wrench_client(body_name)

        body_name = 'robot::Link_innerfinger'
        clear_body_wrench_client(body_name)

        
        body_name = 'robot::Link_exterfinger'
        clear_body_wrench_client(body_name)
        
        hand.set_named_target('handopen')
        hand.go()
        rospy.sleep(1)
        hand.stop()
        
        raw_input("Gripper released, Press Enter to Continue>>>>>>")

        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -80), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 60, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        #joint_positions =  [0, 2.530729, 0]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.2617995, 2.617995, -0.2617995]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.523599, 2.530729, -0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions = [-0.785399, 2.356196, -0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [-0.785399, 2.356196, 1.570797] 
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions = [-0.785399, 1.570797, 2.356196, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [-0.523599, 1.308998, 2.356196]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.523599, 1.047198, 2.094396]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.2617995, 0.785399, 2.094396]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.2617995, 0.523599, 1.832597]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0,  0.2617995, 1.832597]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions = [0, 0, 1.570797, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [0.2617995,  -0.523599, 1.308998]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.2617995, -0.785399, 1.047198]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.523599, -1.047198, 1.047198]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.523599, -1.308998, 0.785399]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0.785399, -1.570797, 0.785399] 
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        joint_positions = [0.785399, -2.356196, 0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        #joint_positions = [0.523599, -2.617995, 0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [0, -2.617995, 0]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.2617995, -2.356196, 0.2617995]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #joint_positions = [-0.523599, -2.094396, 0.523599]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)

        #arm.set_named_target('StartState')
        #arm.go()
        #rospy.sleep(2)

        arm.stop()

        raw_input("Ready to go home, Press Enter to Continue!!!")




        #gripper.set_joint_value_target([0.01])
        #gripper.go()
        #rospy.sleep(1)
         
        
        joint_positions = [-0.523599, -2.094396, 0.523599, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)
        arm.stop()
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    
    try:
        MoveItBigmamaDemo()
    except rospy.ROSInterruptException:
        pass