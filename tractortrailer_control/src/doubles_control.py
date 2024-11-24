#!/usr/bin/env python
import rospy
import roslib
import os
import math
import numpy as np
import random
import csv
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from std_srvs.srv import Empty

import os
import numpy as np
import pandas as pd
import random


#-------------------------------------------------------------------------------------------------------------------
# 						Setup
#-------------------------------------------------------------------------------------------------------------------
	
def euler_from_quaternion(x, y, z, w):
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)
	
	t2 = +2.0 * (w * y - z *x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)
	
	return roll_x, pitch_y, yaw_z	# in radians
	
	
def quaternion_from_euler(yaw, pitch, roll):
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	
	return [qx, qy, qz, qw]
	
	
class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class trailer:
    def __init__(self, x, y, z, roll, pitch, yaw, vel, loop, toc):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.vel = vel
        self.loop = loop
        self.toc = toc
	
class trailer2:
    def __init__(self, x, y, z, roll, pitch, yaw, vel, loop, toc):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.vel = vel
        self.loop = loop
        self.toc = toc
	
class tractor:
    def __init__(self, x, y, z, roll, pitch, yaw, loop, toc):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.loop = loop
        self.toc = toc
		
class imu:
    def __init__(self, x, y, z, roll, pitch, yaw, loop, toc):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.loop = loop
        self.toc = toc
		
class dolly:
    def __init__(self, x, y, z, roll, pitch, yaw, loop, toc):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.loop = loop
        self.toc = toc


class TractorTrailerLinks:
    _blockListDict = {
        'Trailer_Hitch': Block('double_trailer_53', 'Trailer_Hitch'),
        'Tractor_Body': Block('double_trailer_53', 'Tractor_Body'),	
        'Dolly_Body': Block('double_trailer_53', 'Dolly_Body'),	
        'second_trailer': Block('double_trailer_53', 'second_trailer'),	
        'rear_imu': Block('double_trailer_53', 'rear_imu'),	
    }
	
    def show_gazebo_links(self):
        try:
            link_coordinates = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            for block in self._blockListDict.values():
                blockName = str(block._name)
                resp_coordinates = link_coordinates(str(block._relative_entity_name), 'world')
                
                # Create objects for each body's coordinates
                if block._relative_entity_name == 'Trailer_Hitch':
                    [roll, pitch, yaw] = euler_from_quaternion(resp_coordinates.link_state.pose.orientation.x, resp_coordinates.link_state.pose.orientation.y, resp_coordinates.link_state.pose.orientation.z, resp_coordinates.link_state.pose.orientation.w)
                    try:
                        trailer.loop = time.time() - trailer.toc
                        dvx = trailer.x - resp_coordinates.link_state.pose.position.x
                        dvy = trailer.y - resp_coordinates.link_state.pose.position.y
                        trailer.vel = ((dvx**2 + dvy**2)**0.5) / trailer.loop
                    except:
                        trailer.vel = 0
                        trailer.loop = 0.1
                    trailer.x = resp_coordinates.link_state.pose.position.x
                    trailer.y = resp_coordinates.link_state.pose.position.y
                    trailer.z = resp_coordinates.link_state.pose.position.z
                    trailer.roll = roll
                    trailer.pitch = pitch
                    trailer.yaw = yaw
                    trailer.toc = time.time()
					 
                if block._relative_entity_name == 'Tractor_Body':
                    [roll, pitch, yaw] = euler_from_quaternion(resp_coordinates.link_state.pose.orientation.x, resp_coordinates.link_state.pose.orientation.y, resp_coordinates.link_state.pose.orientation.z, resp_coordinates.link_state.pose.orientation.w)
                    try:
                        tractor.loop = time.time() - tractor.toc
                        dvx = tractor.x - resp_coordinates.link_state.pose.position.x
                        dvy = tractor.y - resp_coordinates.link_state.pose.position.y
                        tractor.vel = ((dvx**2 + dvy**2)**0.5) / tractor.loop
                    except:
                        tractor.vel = 0
                        tractor.loop = 0.1
                    tractor.x = resp_coordinates.link_state.pose.position.x
                    tractor.y = resp_coordinates.link_state.pose.position.y
                    tractor.z = resp_coordinates.link_state.pose.position.z
                    tractor.roll = roll
                    tractor.pitch = pitch
                    tractor.yaw = yaw
                    tractor.toc = time.time()
					
                if block._relative_entity_name == 'rear_imu':
                    [roll, pitch, yaw] = euler_from_quaternion(resp_coordinates.link_state.pose.orientation.x, resp_coordinates.link_state.pose.orientation.y, resp_coordinates.link_state.pose.orientation.z, resp_coordinates.link_state.pose.orientation.w)
                    try:
                        imu.loop = time.time() - imu.toc
                        dvx = imu.x - resp_coordinates.link_state.pose.position.x
                        dvy = imu.y - resp_coordinates.link_state.pose.position.y
                        imu.vel = ((dvx**2 + dvy**2)**0.5) / imu.loop
                    except:
                        imu.loop = 0.1
                        imu.vel = 0
                    imu.x = resp_coordinates.link_state.pose.position.x
                    imu.y = resp_coordinates.link_state.pose.position.y
                    imu.z = resp_coordinates.link_state.pose.position.z
                    imu.roll = roll
                    imu.pitch = pitch
                    imu.yaw = yaw
                    imu.toc = time.time()

                if block._relative_entity_name == 'second_trailer':
                    [roll, pitch, yaw] = euler_from_quaternion(resp_coordinates.link_state.pose.orientation.x, resp_coordinates.link_state.pose.orientation.y, resp_coordinates.link_state.pose.orientation.z, resp_coordinates.link_state.pose.orientation.w)
                    try:
                        trailer2.loop = time.time() - trailer2.toc
                        dvx = trailer2.x - resp_coordinates.link_state.pose.position.x
                        dvy = trailer2.y - resp_coordinates.link_state.pose.position.y
                        trailer2.vel = ((dvx**2 + dvy**2)**0.5) / trailer2.loop
                    except:
                        trailer2.loop = 0.1
                        trailer2.vel = 0
                    trailer2.x = resp_coordinates.link_state.pose.position.x
                    trailer2.y = resp_coordinates.link_state.pose.position.y
                    trailer2.z = resp_coordinates.link_state.pose.position.z
                    trailer2.roll = roll
                    trailer2.pitch = pitch
                    trailer2.yaw = yaw
                    trailer2.toc = time.time()

                if block._relative_entity_name == 'Dolly_Body':
                    [roll, pitch, yaw] = euler_from_quaternion(resp_coordinates.link_state.pose.orientation.x, resp_coordinates.link_state.pose.orientation.y, resp_coordinates.link_state.pose.orientation.z, resp_coordinates.link_state.pose.orientation.w)
                    try:
                        dolly.loop = time.time() - dolly.toc
                        dvx = dolly.x - resp_coordinates.link_state.pose.position.x
                        dvy = dolly.y - resp_coordinates.link_state.pose.position.y
                        dolly.vel = ((dvx**2 + dvy**2)**0.5) / dolly.loop
                    except:
                        dolly.loop = 0.1
                        dolly.vel = 0
                    dolly.x = resp_coordinates.link_state.pose.position.x
                    dolly.y = resp_coordinates.link_state.pose.position.y
                    dolly.z = resp_coordinates.link_state.pose.position.z
                    dolly.roll = roll
                    dolly.pitch = pitch
                    dolly.yaw = yaw
                    dolly.toc = time.time()

        except rospy.ServiceException as e:
            rospy.loginfo("Get Link State service call failed: {0}".format(e))


#-------------------------------------------------------------------------------------------------------------------
# 						                    Simulation Stuff
#-------------------------------------------------------------------------------------------------------------------

def reset_pos(scenario):
	
    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    
    if scenario == 0:
        y = 0
        x = 0
        yaw = 0
        [qx, qy, qz, qw] = quaternion_from_euler(yaw, 0, 0)
        position = "position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.25}"
        orientation = "orientation: {x: " + str(qx) + ", y: " + str(qy) + ", z: " + str(qz) + ", w: " + str(qw) + "}"

    for n in range(0,2):
        command = "rosservice call /gazebo/set_model_configuration '{model_name: double_trailer_53, joint_names: ['Fifth_Wheel','dolly_Fifth_Wheel','Dolly_Hitch_Joint'], joint_positions: [0,0,0]}'"
        os.system(command)

        
        command = "rosservice call /gazebo/set_model_state '{model_state: {model_name: double_trailer_53, pose: {" + position + ", " + orientation + "}, twist: { linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}, reference_frame: world}}'"
        os.system(command)
			
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
	

def calculate_speed(speed):

	# Spin the wheels based on error of velocity and desired velocity
    speed = 60

    if tractor.vel*-1 < -2:
        speed = speed
    elif tractor.vel*-1 > -2:
        speed = -speed
    else:
        speed = -60
   
    return(speed)


#-------------------------------------------------------------------------------------------------------------------
# 						                        Main
#-------------------------------------------------------------------------------------------------------------------

def main():
    # Create publisher instance
    driv_wheel_pub = rospy.Publisher('/double_trailer_53/Driv_velocity_controller/command', Float64, queue_size=10)
    pass_wheel_pub = rospy.Publisher('/double_trailer_53/Pass_velocity_controller/command', Float64, queue_size=10)
    dolly_driv_pub = rospy.Publisher('/double_trailer_53/Dolly_driv_velocity_controller/command', Float64, queue_size=10)
    dolly_pass_pub = rospy.Publisher('/double_trailer_53/Dolly_pass_velocity_controller/command', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/double_trailer_53/Axle_Steer_position_controller/command', Float64, queue_size=10)
    rospy.init_node('Virtual_Driver', anonymous=True)
    
    # Initialize Variables
    rate = rospy.Rate(10) #Hz	
    ttlinks = TractorTrailerLinks()
    n = 0
    reset_pos(0)
    ooga = 1
	
    while not rospy.is_shutdown():
        
        # Update the pose of the links
        ttlinks.show_gazebo_links()
        
        # Reset position of tractor trailer just in case
        #reset_pos(0)
        
        # Run Simulation
        steering_pub.publish(-0.0)
        print(tractor.vel)

        if tractor.vel > 2 or ooga != 1:
            print('Torque that bad boy')
            force = 45359*4 # Newtons
            torque_x = 0
            torque_z = force * 1.15 #Apply at center of driv wheel
            torque_y = force * 0.5 #Apply where tire meets ground
            torque = "{x: " + str(torque_x) + ", y: " + str(torque_y) + ", z: " + str(-torque_z) + "}"
            time = str(math.floor(tractor.loop *(10**9)))
            print(time)
            print(torque)
            command = "rosservice call /gazebo/apply_body_wrench '{body_name: Dolly_Body, wrench: {force: " + torque + "}, start_time: 1, duration: " + time + "}'"
            os.system(command)
            #command = "rosservice call /gazebo/apply_body_wrench '{body_name: Dolly_Body, wrench: {torque: " + torque + "}, start_time: 1, duration: 100000000}'"
            #os.system(command)
            #torque = "{x: " + str(torque_x) + ", y: " + str(-torque_y) + ", z: " + str(torque_z) + "}"
            #print(torque)
            #command = "rosservice call /gazebo/apply_body_wrench '{body_name: Dolly_Body, wrench: {torque: " + torque + "}, start_time: 1, duration: 100000000}'"
            os.system(command)
            driv_wheel_pub.publish(0)
            pass_wheel_pub.publish(0)
            dolly_driv_pub.publish(0)
            dolly_pass_pub.publish(0)
            ooga = 0
        else:
            driv_wheel_pub.publish(60)
            pass_wheel_pub.publish(60)
            dolly_driv_pub.publish(60)
            dolly_pass_pub.publish(60)
            
        rate.sleep()

	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
