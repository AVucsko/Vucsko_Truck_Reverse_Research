#!/usr/bin/env python
import rospy
import roslib
import os
import math
import numpy as np
import random
import csv
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
	def __init__(self, x, y, z, roll, pitch, yaw, vel):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.vel = vel
		
class tractor:
	def __init__(self, x, y, z, roll, pitch, yaw):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		
class distance:
    def __init__(self, current, currentx, currenty, previous, previousx, previousy):
        self.current = current
        self.currentx = currentx
        self.currenty = currenty
        self.previous = previous
        self.previousx = previousx
        self.previousy = previousy
			
		
class phi_trailer:
	def __init__(self, yaw, RB, RU, RV, VE, LV, LU, LB):
		self.yaw = yaw
		self.RB = RB
		self.RU = RU
		self.RV = RV
		self.VE = VE
		self.LV = LV
		self.LU = LU
		self.LB = LB	
		
class phi_tractor:
	def __init__(self, yaw, NE, ZR, PO):
		self.yaw = yaw
		self.NE = NE
		self.ZR = ZR
		self.PO = PO
		
class x_pos:
	def __init__(self, pos, LE, LC, CE, RC, RI):
		self.pos = pos
		self.LE = LE
		self.LC = LC
		self.CE = CE
		self.RC = RC
		self.RI = RI	


class TractorTrailerLinks:
	_blockListDict = {
		'Trailer_Hitch': Block('TractorTrailer_0', 'Trailer_Hitch'),
		'Tractor_Body': Block('TractorTrailer_0', 'Tractor_Body'),	
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
					trailer.x = resp_coordinates.link_state.pose.position.x
					trailer.y = resp_coordinates.link_state.pose.position.y
					trailer.z = resp_coordinates.link_state.pose.position.z
					trailer.roll = roll
					trailer.pitch = pitch
					trailer.yaw = yaw
					 
				if block._relative_entity_name == 'Tractor_Body':
					[roll, pitch, yaw] = euler_from_quaternion(resp_coordinates.link_state.pose.orientation.x, resp_coordinates.link_state.pose.orientation.y, resp_coordinates.link_state.pose.orientation.z, resp_coordinates.link_state.pose.orientation.w)
					tractor.x = resp_coordinates.link_state.pose.position.x
					tractor.y = resp_coordinates.link_state.pose.position.y
					tractor.z = resp_coordinates.link_state.pose.position.z
					tractor.roll = roll
					tractor.pitch = pitch
					tractor.yaw = yaw
		except rospy.ServiceException as e:
			rospy.loginfo("Get Link State service call failed: {0}".format(e))



#-------------------------------------------------------------------------------------------------------------------
# 						Calculating Steering Angle
#-------------------------------------------------------------------------------------------------------------------	
class controllers:
	def __init__(self, weights, best, secondbest, fitness, avgfit):
		self.weights = weights[3][5]
		
class previous_it:
    def __init__(self, theta, scenario):
        self.theta = theta
        self.scenario = scenario

def decide_controller():
	# scenario == 0: Fine Controller
	# scenario == 1: HighX Controller
	# scenario == 2: HighYaw Controller
	
	
	if previous_it.scenario == 2:
	    if abs(trailer.x) <= 5 and abs(trailer.yaw) <= 0.2:
		    scenario = 0
	    elif (abs(trailer.yaw) <= 0.2) and (abs(tractor.yaw) <= 0.2):
		    scenario = 1
	    else:
		    scenario = 2
	if previous_it.scenario == 1:
	    if abs(trailer.x) <= 5 and abs(trailer.yaw) <= 0.2:
		    scenario = 0
	    elif abs(trailer.yaw) <= 1.4:
		    scenario = 1
	    else:
		    scenario = 2
	else:
	    if abs(trailer.x) <= 5 and abs(trailer.yaw) <= 0.2:
		    scenario = 0
	    elif abs(trailer.yaw) <= 0.6:
		    scenario = 1
	    else:
		    scenario = 2
	
	previous_it.scenario = scenario
	return scenario


def rate_limit(theta):
    # Currently limiting the steering command to 360 deg/s -- In line with rereax limiters at this speed
    if abs((abs(theta) - abs(previous_it.theta))) >= 0.0628:
        if theta > previous_it.theta:
            theta = previous_it.theta + 0.0628
        else:
            theta = previous_it.theta - 0.0628
            
    previous_it.theta = theta
    
    return(theta)
    
    
def calculate_theta():

	# Decide which controller to use
	scenario = decide_controller()
	
	# Gather the necessary weights
	w1 = controllers.weights[scenario][0]
	w2 = controllers.weights[scenario][1]
	w3 = controllers.weights[scenario][2]
	w4 = controllers.weights[scenario][3]
	w5 = controllers.weights[scenario][4]
	
	# Neurons
	z1 = np.tanh(tractor.yaw * w1 + trailer.yaw * w2)
	z2 = np.tanh(trailer.x * w3)
	z3 = np.tanh(z1*w4 + z2*w5)
	
	# Rate Limit Output Command
	output = rate_limit(z3)
	return(output)
	
def sliding_mode(x_prev, s_prev, v):
    # Define Variables
    k1 = 
    q = 
    l = -0.2 # Represents the distance of rear axle to hitch which is actually 0 but that destroys the math
    L2 = 15.8 #meters
    # Angle
    psi = trailer.yaw - tractor.yaw
    
    # Derive Errors
    error_dot = (trailer.x - x_prev) / 0.1
    
    # Sliding surface
    s = error_dot + k1*error
    s_dot = (s-s_prev)/0.1
    if s < 0:
        s_dot = -q*-1
        c = L2*q*-1/(v**2*l*math.cos(psi)*math.cos(error_theta))
    else:
        s_dot = -q
        c = L2*q*1/(v**2*l*math.cos(psi)*math.cos(error_theta))
        
    # Desired Steering Angle
    a = -tan(psi)/l
    b = (k1*L2*math.sin(error_theta))/(v*l*math.cos(psi)*math.cos(error_theta))
    
    theta = math.atan2(a + b + c)
	
#-------------------------------------------------------------------------------------------------------------------
# 						                    Simulation Stuff
#-------------------------------------------------------------------------------------------------------------------

def reset_pos(scenario):
	
	pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
	
	if scenario == 0:
		y = -50
		#x = random.uniform(-50, 50)
		#yaw = random.uniform(-1.57, 1.57)
		x = 0
		yaw = 0
		[qx, qy, qz, qw] = quaternion_from_euler(yaw, 0, 0)
		position = "position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.25}"
		orientation = "orientation: {x: " + str(qx) + ", y: " + str(qy) + ", z: " + str(qz) + ", w: " + str(qw) + "}"
	
		command = "rosservice call /gazebo/set_model_state '{model_state: {model_name: TractorTrailer_0, pose: {" + position + ", " + orientation + "}, twist: { linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}, reference_frame: world}}'"
		os.system(command)
		
		command = "rosservice call /gazebo/set_model_configuration '{model_name: TractorTrailer_0, joint_names: ['Fifth_Wheel'], joint_positions: [0] }'"
		os.system(command)
	
	elif scenario == 1:
		y = -50
		x = 20
		yaw = 0
		[qx, qy, qz, qw] = quaternion_from_euler(yaw, 0, 0)
		position = "position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.25}"
		orientation = "orientation: {x: " + str(qx) + ", y: " + str(qy) + ", z: " + str(qz) + ", w: " + str(qw) + "}"
		
		command = "rosservice call /gazebo/set_model_state '{model_state: {model_name: TractorTrailer_0, pose: {" + position + ", " + orientation + "}, twist: { linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}, reference_frame: world}}'"
		os.system(command)
		
		command = "rosservice call /gazebo/set_model_configuration '{model_name: TractorTrailer_0, joint_names: ['Fifth_Wheel'], joint_positions: [0] }'"
		os.system(command)
	
	elif scenario == 2:
		y = -50
		x = -20
		yaw = 0
		[qx, qy, qz, qw] = quaternion_from_euler(yaw, 0, 0)
		position = "position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.25}"
		orientation = "orientation: {x: " + str(qx) + ", y: " + str(qy) + ", z: " + str(qz) + ", w: " + str(qw) + "}"
		
		command = "rosservice call /gazebo/set_model_state '{model_state: {model_name: TractorTrailer_0, pose: {" + position + ", " + orientation + "}, twist: { linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}, reference_frame: world}}'"
		os.system(command)
		
		command = "rosservice call /gazebo/set_model_configuration '{model_name: TractorTrailer_0, joint_names: ['Fifth_Wheel'], joint_positions: [0] }'"
		os.system(command)
	
	elif scenario == 3:
		y = -50
		x = 50
		yaw = 1.57
		[qx, qy, qz, qw] = quaternion_from_euler(yaw, 0, 0)
		position = "position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.25}"
		orientation = "orientation: {x: " + str(qx) + ", y: " + str(qy) + ", z: " + str(qz) + ", w: " + str(qw) + "}"
		
		command = "rosservice call /gazebo/set_model_state '{model_state: {model_name: TractorTrailer_0, pose: {" + position + ", " + orientation + "}, twist: { linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}, reference_frame: world}}'"
		os.system(command)
		
		command = "rosservice call /gazebo/set_model_configuration '{model_name: TractorTrailer_0, joint_names: ['Fifth_Wheel'], joint_positions: [0] }'"
		os.system(command)
	
	elif scenario == 4:
		y = -50
		x = -50
		yaw = -1.57
		[qx, qy, qz, qw] = quaternion_from_euler(yaw, 0, 0)
		position = "position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.25}"
		orientation = "orientation: {x: " + str(qx) + ", y: " + str(qy) + ", z: " + str(qz) + ", w: " + str(qw) + "}"
		
		command = "rosservice call /gazebo/set_model_state '{model_state: {model_name: TractorTrailer_0, pose: {" + position + ", " + orientation + "}, twist: { linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}, reference_frame: world}}'"
		os.system(command)
		
		command = "rosservice call /gazebo/set_model_configuration '{model_name: TractorTrailer_0, joint_names: ['Fifth_Wheel'], joint_positions: [0] }'"
		os.system(command)
			
	unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
	

def calculate_speed(speed):
		
    # Calculate the distance to the origin
    distance.current = ((tractor.x**2) + (tractor.y**2))**0.5
    distance.currentx = tractor.x
    distance.currenty = tractor.y

    # Calculate the velocity of the tractor - Always spits out 0
    try:
        dvx = distance.currentx - distance.previousx
        dvy = distance.currenty - distance.previousy
        vel = ((dvx**2 + dvy**2)**0.5) * 10
    except:
        vel = 1
		
		
    if vel > 0:
        vel = vel * -1
	    
    distance.previous = distance.current
    distance.previousx = distance.currentx
    distance.previousy = distance.currenty


	# Spin the wheels based on error of velocity and desired velocity
    speed = 60

    if vel < -2:
        speed = speed
    elif vel > -2:
        speed = -speed
    else:
        speed = -60
	    
    #print("Velocity: " + str(vel))
   
    return(speed, vel)


#-------------------------------------------------------------------------------------------------------------------
# 						                        Main
#-------------------------------------------------------------------------------------------------------------------

def main():
	# Create publisher instance
	driv_wheel_pub = rospy.Publisher('/TractorTrailer_0/Driv_velocity_controller/command', Float64, queue_size=10)
	pass_wheel_pub = rospy.Publisher('/TractorTrailer_0/Pass_velocity_controller/command', Float64, queue_size=10)
	steering_pub = rospy.Publisher('/TractorTrailer_0/Axle_Steer_position_controller/command', Float64, queue_size=10)
	rospy.init_node('Virtual_Driver', anonymous=True)
	
	# Initialize Variables
	rate = rospy.Rate(10) #Hz
	speed = -10
	theta = 0
	generations = 0
	numsteps = 1000
	previous_it.theta = 0
	previous_it.scenario = 0
	x_prev = 0
	
	# Initialize Controllers - Fine, HighX, HighYaw
	#controllers.weights = np.array([[-0.23021679, 1.08676092, -0.09843739, -3.9608538, -1.01370531], [-0.1221, 1.12792065, -0.06235775, -2.5353074, -1.17929686], [ 1.10179206, -0.10262596, -0.30135633, 5.00553137, 4.6864815 ]]) -- This was the original controller but the new one (below) is better
	controllers.weights = np.array([[-0.23021679, 1.08676092, -0.09843739, -3.9608538, -1.01370531], [-0.13244344, 0.61450203, -0.06354845, -3.70897267, -1.17183854], [ 1.10179206, -0.10262596, -0.30135633, 5.00553137, 4.6864815 ]])
	
	
	while not rospy.is_shutdown():
		
		# Update the pose of the links
		ttlinks = TractorTrailerLinks()
		ttlinks.show_gazebo_links()
		
		# Reset position of tractor trailer just in case
		reset_pos(0)
		
		# Open Files
		#actual_file = open("X_Error_4.txt", "a")
		#error1_file = open("Y_Error_4.txt", "a")
	    #error2_file = open("Yaw_Error_4.txt", "a")
		
		# Run Simulation
		for y in range(1, 2):
			reset_pos(y)
			for n in range(0, numsteps):
				
				# Update the pose of the links
				ttlinks = TractorTrailerLinks()
				ttlinks.show_gazebo_links()
				
				# Desired command
				speed, vel = calculate_speed(speed)
				theta = calculate_theta()
				theta = sliding_mode(x_prev)
				
				# Publish movement command and wait for next loop
				driv_wheel_pub.publish(speed)
				pass_wheel_pub.publish(speed)
				steering_pub.publish(theta)
				rate.sleep()
				
				print((tractor.yaw - trailer.yaw)*180/3.14)
				
				# Trailer Lat Error
				lat_error = 16*math.tan(trailer.yaw) + trailer.x
				x_prev = trailer.x
				
				# Write info to file
				#actual_file.write(str(trailer.x) + "\n")
				#error1_file.write(str(trailer.y) + "\n")
				#error2_file.write(str(trailer.yaw) + "\n")
				
				rate.sleep()

	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
