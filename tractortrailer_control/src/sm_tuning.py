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
# 						Genetic Algo Stuff
#-------------------------------------------------------------------------------------------------------------------
		
class chromosomes:
	def __init__(self, weights, best, secondbest, fitness, avgfit):
		self.weights = weights[10][2]
		self.best = best[1][2]
		self.secondbest = secondbest[1][2]
		self.fitness = fitness[10][5]
		self.avgfit = avgfit[10][1]


def chromosomes_init():
	chromosomes.weights = np.array([[0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01]])
	
	chromosomes.fitness = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
	
	chromosomes.avgfit = [[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]]
	chromosomes.best = np.array([0.18958181, 0.18164382, 8.8958181, 0.088164382])
	#chromosomes.best = np.array([0.25866081, 0.53591704, 9.66983913, 4.62051466])
	chromosomes.secondbest = np.array([0.87414368, 0.34819866, 0.78958181*100, 0.18164382*100])


def mutation():
	chance = random.randint(1,10)
	if chance > 6:
	    mut = 2
	elif chance > 3:
		mut = 1
	else:
		mut = 0
		
	return(mut)

def create_chromosomes():
	#Weight1 - k1
	#Weight2 - q
	
	
	# Because I am keeping the best result from the last set, I'm at risk of reaching a local max not a global max. Will test and see if it's good enough regardless
	chromosomes.weights[0] = chromosomes.best
	x = 1
	
	while x < 10:
	    mut = mutation()
	    if mut == 2:
		    w1 = chromosomes.best[0]
	    elif mut == 1:
	        w1 = chromosomes.secondbest[0]
	    else:
		    w1 = chromosomes.best[0] + random.uniform(-0.2, 0.2)
	    
	    if w1 > 100:
		    w1 = 100
	    elif w1 < 0.01:
		    w1 = 0.01
	    
	    
	    mut = mutation()
	    if mut == 2:
		    w2 = chromosomes.best[1]
	    elif mut == 1:
	        w2 = chromosomes.secondbest[1]
	    else:
		    w2 = chromosomes.best[1] + random.uniform(-0.2, 0.2)
	    if w2 > 100:
		    w2 = 100
	    elif w2 < 0.01:
		    w2 = 0.01
		    
	    mut = mutation()
	    if mut == 2:
		    w3 = chromosomes.best[2]
	    elif mut == 1:
	        w3 = chromosomes.secondbest[2]
	    else:
		    w3 = chromosomes.best[2] + random.uniform(-1, 1)
	    
	    if w3 > 100:
		    w3 = 100
	    elif w3 < 0.01:
		    w3 = 0.01
	    
	    
	    mut = mutation()
	    if mut == 2:
		    w4 = chromosomes.best[3]
	    elif mut == 1:
	        w4 = chromosomes.secondbest[3]
	    else:
		    w4 = chromosomes.best[3] + random.uniform(-1, 1)
	    if w4 > 100:
		    w4 = 100
	    elif w4 < 0.01:
		    w4 = 0.01

	    
	    chromosomes.weights[x][0] = w1
	    chromosomes.weights[x][1] = w2
	    chromosomes.weights[x][2] = w3
	    chromosomes.weights[x][3] = w4
	    
	    if not(w1 == chromosomes.best[0] and w2 == chromosomes.best[1]):
	        x = x + 1
	    else:
	        x = x + 1
	        
	print(chromosomes.weights)
		
	   

	
def fitness(x,n):
	# Convert Yaw values to degrees so we can safely square
	tractoryaw = tractor.yaw * 180 / 3.14
	traileryaw = trailer.yaw * 180 / 3.14
	if n > 100:
	    fit = 1/((traileryaw**2) + 100 * (trailer.x**2) + 0.01*n + 1)
	else:
		fit = 0
	if fit > chromosomes.fitness[x][0]:
		chromosomes.fitness[x][0] = fit
		
	return(fit)
	
def findsecondfit():
	test = 0
	smaller = 0
	answer = 0
	
	for i in range(0,9):
		test = chromosomes.avgfit[i][0]
		for x in range(0,9):
			if test < chromosomes.avgfit[x][0]:
				smaller = smaller + 1
		
		if smaller == 1:
			answer = test
			
		smaller = 0
		
	return(answer)
	

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

def rate_limit(theta):
    # Currently limiting the steering command to 360 deg/s -- In line with rereax limiters at this speed
    if abs((abs(theta) - abs(previous_it.theta))) >= 0.0628:
        if theta > previous_it.theta:
            theta = previous_it.theta + 0.0628
        else:
            theta = previous_it.theta - 0.0628
            
    previous_it.theta = theta
    
    return(theta)
    
    
	
def sliding_mode(k1x, qx, k1p, qp, x_prev, psi_prev, s_prev_x, s_prev_p, v):
    # Define Variables
    #k1 = 
    #q = 
    #print(k1, q)
    #l = 0.1 # Represents the distance of rear axle to hitch which is actually 0 but that destroys the math
    L2 = 15.8 #meters
    # Angle
    psi = trailer.yaw - tractor.yaw
    error_theta = trailer.yaw
    error = trailer.x
    
    # Derive Errors
    error_dot = (trailer.x - x_prev) / 0.1
    psi_dot = (psi - psi_prev) / 0.1
    
    # Sliding surface - Lat Error
    s_x = error_dot + k1x*error
    s_dot_x = (s_x-s_prev_x)/0.1
    if s_x < 0:
        s_dot_x = qx*-1
        #c = L2*q*-1/(v**2*l*math.cos(psi)*math.cos(error_theta))
    else:
        s_dot_x = qx
        #c = L2*q*1/(v**2*l*math.cos(psi)*math.cos(error_theta))
        
    s_p = psi_dot + k1p*psi
    s_dot_p = (s_p-s_prev_p)/0.1
    if s_p < 0:
        s_dot_p = -qp*-1
    else:
        s_dot_p = -qp
        
    # Desired Steering Angle
    #a = -math.tan(psi)/l
    #b = (k1*L2*math.sin(error_theta))/(v*l*math.cos(psi)*math.cos(error_theta))
    #theta = math.atan2(a + b + c,1)
    #print(s_dot_x, s_dot_p)
    #theta = s_dot_x + s_dot_p
    #print(s_x, s_p)
    theta = s_x + s_p*-1
    
    
    # Trying to move between modes
    #print(math.atan2(trailer.y, trailer.x)+1.07, trailer.yaw)
    if math.atan2(trailer.y, trailer.x)+1.07 > trailer.yaw - 0.4 and  math.atan2(trailer.y, trailer.x)+1.07 < trailer.yaw + 0.4:
        theta = s_x + s_p*-1
    else:
        theta = s_dot_x + s_dot_p
        
    return(theta, s_x, s_p, psi)
	
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
	theta_sm = 0
	generations = 0
	numsteps = 500
	previous_it.theta = 0
	previous_it.scenario = 0
	x_prev = 0
	psi_prev = 0
	s_x = 0
	s_p = 0
	chromosomes_init()
	
	x_file = open("X_Error_sm_3.txt", "a")
	y_file = open("Y_Error_sm_3.txt", "a")
	yaw_file = open("Yaw_Error_sm_3.txt", "a")
	
	
	
	while not rospy.is_shutdown():
		
		# Update the pose of the links
		ttlinks = TractorTrailerLinks()
		ttlinks.show_gazebo_links()
		
		# Reset position of tractor trailer just in case
		reset_pos(0)
		
		create_chromosomes()
		
		# Open Files
		#actual_file = open("X_Error_4.txt", "a")
		#error1_file = open("Y_Error_4.txt", "a")
	    #error2_file = open("Yaw_Error_4.txt", "a")
	    
	    # Open Files
		weights_file = open("Weights_Trailer_SM_2.txt", "a")
		fitness_file = open("Fitness_Trailer_SM_2.txt", "a")
		
		# Run Simulation
		print("Generation: " + str(generations))
		for x in range(0, 10):
		    for y in range(2, 3):
			    reset_pos(y)
			    for n in range(0, numsteps):
				    
				    # Update the pose of the links
				    ttlinks = TractorTrailerLinks()
				    ttlinks.show_gazebo_links()
				    
				    # Desired command
				    speed, vel = calculate_speed(speed)
				    #k1 = random.uniform(-.0001, .0001)
				    #q = random.uniform(0, .0001)
				    #k1 = 5.1313 * 10**-3 
				    #q = 2.62109 * 10**-3
				    theta_sm, s_x, s_p, psi = sliding_mode(chromosomes.weights[x][0], chromosomes.weights[x][1], chromosomes.weights[x][2], chromosomes.weights[x][3], x_prev, psi_prev, s_x, s_p, vel)
				    
				    # Publish movement command and wait for next loop
				    driv_wheel_pub.publish(speed)
				    pass_wheel_pub.publish(speed)
				    steering_pub.publish(theta_sm)
				    
				    # Calculate fitness value
				    fit = fitness(x, n)
				    chromosomes.fitness[x][y] = fit
				    rate.sleep()
				    
				    # Trailer Lat Error
				    lat_error = 16*math.tan(trailer.yaw) + trailer.x
				    x_prev = trailer.x
				    psi_prev = psi
				    
				    # Write info to file
				    x_file.write(str(trailer.x) + "\n")
				    y_file.write(str(trailer.y) + "\n")
				    yaw_file.write(str(trailer.yaw) + "\n")
				    
				    
		    fit = (chromosomes.fitness[x][0] + chromosomes.fitness[x][1] + chromosomes.fitness[x][2] + chromosomes.fitness[x][3] + chromosomes.fitness[x][4])/5
		    chromosomes.avgfit[x][0] = fit
		    print(fit)
			
		# Find best and second best performers
		chromosomes_copy = chromosomes
		fit_second = findsecondfit()
		fit_max = max(chromosomes.avgfit)
		print("Fit_max: " + str(fit_max))
				
		# Save weights from best and second best performers
		for i in range(0, 10):
			#print("Fitmax: " + str(fit_max))
			print(chromosomes.avgfit[i][0])
			if chromosomes.avgfit[i][0] == fit_max[0]:
				chromosomes.best = chromosomes.weights[i]
				print("Best Weights:")
				print(chromosomes.best)
			if chromosomes.avgfit[i][0] == fit_second:
				chromosomes.secondbest = chromosomes.weights[i]
				print("Second Best Weights:")
				print(chromosomes.secondbest)
				
		# Write important info to files
		weights_file.write(str(chromosomes.best) + "\n")
		fitness_file.write(str(fit_max) + "\n")
		
		
	# Close files
	weights_file.close
	fitness_file.close

	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
