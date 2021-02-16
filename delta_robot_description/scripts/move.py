#!/usr/bin/env python
#-------- Libraries --------#
import math
import numpy as np
#---------------------------#
#-------- ROS Libraries --------#
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import geometry_msgs.msg
import tf

import time
#-------------------------------#
#-------- Node initialization --------#
rospy.init_node('delta_robot') # Node initialization in ROS
broadcaster = tf.TransformBroadcaster()
odom_trans = geometry_msgs.msg.TransformStamped()
joint_state = JointState()
odom_trans.header.frame_id = 'odom'
odom_trans.child_frame_id = 'base_link'
joint_state.header = Header()
joint_state.name = ['base_to_arm_1', 'base_to_arm_2', 'base_to_arm_3',
'down_arm_1_to_arm_1', 'down_arm_2_to_arm_2',
'down_arm_3_to_arm_3','tool0_to_down_arm_1']
#-------------------------------------#
#-------- Variables --------#
L = 0.300 # Upper legs length
l = 0.800 # Lower legs length
wb = 0.180 # Planar distance from {0} to near base side
wp = 0.030 # Planar distance from {p} to near platform side
up = 0.035 # Planar distance from {p} to a platform vertex
sp = 0.100 # Platform equilateral triangle side
E = np.zeros(3) #Variable for calculate angle
F = np.zeros(3) #Variable for calculate angle
G = np.zeros(3) #Variable for calculate angle
t1 = np.zeros(3) #Variable for calculate angle
t2 = np.zeros(3) #Variable for calculate angle
angle_upper_arm_rad_1 = np.zeros(3) #First solution angle of motors in rad
angle_upper_arm_rad_2 = np.zeros(3) #Second solution angle of motors in rad
angle_upper_arm_deg_1 = np.zeros(3) #First solution angle of motors in degrees
angle_upper_arm_deg_2 = np.zeros(3) #Second solution angle of motors in degrees
final_angle_lower_arm1 = np.zeros(1)
final_angle_lower_arm2 = np.zeros(1)
final_angle_lower_arm3 = np.zeros(1)
end_effector = np.zeros(1)
k=0 #Flag for changing coordinates

#---------------------------#
#-------- Function block for calculating angles --------#
def calculate_delta_robot_angle():
 #-------- Calculate a, b, c --------#
 a = wb - up
 b = (sp/2.0) - ((math.sqrt(3.0)/2.0) * wb)
 c = wp - (wb/2.0)
 #-------- Calculate E[0], F[0], G[0] --------#
 E[0] = (2.0 * L) * (y + a)
 F[0] = (2.0 * z) * L
 G[0] = math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(a, 2) + math.pow(L,2) + (2.0 * y * a) - math.pow(l, 2)
 #-------- Calculate E[1], F[1], G[1] --------#
 E[1] = -L * ((math.sqrt(3.0) * (x + b)) + y + c)
 F[1] = 2.0 * z * L
 G[1] = math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c,2) + math.pow(L, 2) + (2.0 * ((x * b)+ (y * c))) - math.pow(l, 2)
 #-------- Calculate E[2], F[2], G[2] --------#
 E[2] = L * ((math.sqrt(3.0) * (x - b)) - y - c)
 F[2] = 2.0 * z * L
 G[2] = math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c,2) + math.pow(L, 2) + (2.0 * (-(x * b) + (y * c))) - math.pow(l, 2)

 #-------- Calculate angle in rad and degrees --------#
 for i in range(0, 3):
     t1[i] = (-F[i] + math.sqrt(math.pow(F[i],2) - math.pow(G[i],2) + math.pow(E[i],2))) /(G[i] - E[i])
     t2[i] = (-F[i] - math.sqrt(math.pow(F[i],2) - math.pow(G[i],2) + math.pow(E[i],2))) /(G[i] - E[i])
     angle_upper_arm_rad_1[i] = 2 * math.atan(t1[i])
     angle_upper_arm_rad_2[i] = 2 * math.atan(t2[i]) #Usada
     angle_upper_arm_deg_1[i] = math.degrees(angle_upper_arm_rad_1[i])
     angle_upper_arm_deg_2[i] = math.degrees(angle_upper_arm_rad_2[i])

#----------------------------------------------------------------#

def calculate_angle_lower_arms():
#####Calculate angle#####
 p0 = np.array([x, y, z])
 p1 = np.array([0, -up, 0])
 p2 = np.array([sp/2, wp, 0])
 p3 = np.array([-sp/2, wp, 0])
 a1 = np.array([0, -wb-L*math.cos(angle_upper_arm_rad_2[0]), - L*math.sin(angle_upper_arm_rad_2[0])])
 a2 = np.array([((math.sqrt(3))/2)*(wb+(L*math.cos(angle_upper_arm_rad_2[1]))), 0.5*(wb+(L*math.cos(angle_upper_arm_rad_2[1]))), -L*math.sin(angle_upper_arm_rad_2[1])])

 a3 = np.array([-((math.sqrt(3))/2)*(wb+(L*math.cos(angle_upper_arm_rad_2[2]))), 0.5*(wb+(L*math.cos(angle_upper_arm_rad_2[2]))), -L*math.sin(angle_upper_arm_rad_2[2])])
 pf1 = p0 + p2
 pf2 = p0 + p3
 pf3 = p0 + p1

 d1_square = ((pf1 - a1)*(pf1 - a1)).sum()
 d2_square = ((pf2 - a2)*(pf2 - a2)).sum()
 d3_square = ((pf3 - a3)*(pf3 - a3)).sum()
 D1 = math.acos((d1_square - math.pow(sp,2)-math.pow(l,2))/((-2)*sp*l))
 alpha1 = math.pi - D1
 beta1 = math.pi - angle_upper_arm_rad_2[0]
 final_angle_lower_arm1[0] = beta1 - alpha1 - (math.pi/2) + 0.15
 D2 = math.acos((d2_square - math.pow(sp,2)-math.pow(l,2))/((-2)*sp*l))
 alpha2 = math.pi - D2
 beta2 = math.pi - angle_upper_arm_rad_2[1]
 final_angle_lower_arm2[0] = beta2 - alpha2 - (math.pi/2)
 D3 = math.acos((d3_square - math.pow(sp,2)-math.pow(l,2))/((-2)*sp*l))
 alpha3 = math.pi - D3
 beta3 = math.pi - angle_upper_arm_rad_2[2]
 final_angle_lower_arm3[0] = beta3 - alpha3 - (math.pi/2) + 0.1
 end_effector[0] = (angle_upper_arm_rad_2[0] + final_angle_lower_arm1[0]) * -1

 #-------- Function block for sending joint state to URDF --------#
def send_joint_state_to_urdf():
    joint_state.header.stamp = rospy.Time.now()
    joint_state.position = [upper_arm_1, upper_arm_2, upper_arm_3, lower_arm_1,lower_arm_2,lower_arm_3, effector]
    odom_trans.header.stamp = rospy.Time.now()
    pub.publish(joint_state)
    # broadcaster.sendTransform((upper_arm_1, upper_arm_2, upper_arm_3), (lower_arm_1,lower_arm_2,lower_arm_3, effector), rospy.Time.now(),'base_link','odom')
    broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(),'base_link','odom')
#----------------------------------------------------------------#

#-------- Definition of publisher --------#
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
#-----------------------------------------#
#-------- Main program --------#
r = rospy.Rate(1000)
#------------------------------#
#-------- Infinite loop --------#
while not rospy.is_shutdown():
#------------------------------#
#-------- Three coordinates changing --------#
    x = 0.0
    y = 0.0
    z = -0.5
    # if k == 0:
    #     x = 0.0 ##Coordinates
    #     y = 0.0
    #     z = -0.5
    # if k == 1:
    #     x = 0.0
    #     y = 0.0
    #     z = -0.7
    # if k == 2:
    #     x = 0.0
    #     y = 0.0
    #     z = -1.0
#--------------------------------------------#
    calculate_delta_robot_angle() #Call function calculate_delta_robot_angle
    calculate_angle_lower_arms()
    upper_arm_1 = angle_upper_arm_rad_2[0]
    upper_arm_2 = angle_upper_arm_rad_2[1]
    upper_arm_3 = angle_upper_arm_rad_2[2]
    lower_arm_1 = final_angle_lower_arm1
    lower_arm_2 = final_angle_lower_arm2
    lower_arm_3 = final_angle_lower_arm3
    effector = end_effector
    send_joint_state_to_urdf() #Call function send_joint_state_to_urdf
    # time.sleep(5)
    # k=k+1
    # if k == 3:
    #     k = 0
    print angle_upper_arm_deg_2
    print angle_upper_arm_rad_2
    print final_angle_lower_arm1 #angulo lower arm
    print final_angle_lower_arm2 #angulo lower arm
    print final_angle_lower_arm3 #angulo lower arm
    print end_effector
    r.sleep()

