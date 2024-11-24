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
		self.weights = weights[10][5]
		self.best = best[1][5]
		self.secondbest = secondbest[1][5]
		self.fitness = fitness[10][5]
		self.avgfit = avgfit[10][1]


def chromosomes_init():
	chromosomes.weights = np.array([[0.02, 0.49, -0.07, -9.89, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
	
	chromosomes.fitness = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
	
	chromosomes.avgfit = [[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]]	

	chromosomes.best = np.array([0.2, -0.2, 0.2, 2, 2])
	chromosomes.best = np.array([-0.2306, 0.7741, 0.0081, -2.341, -2.5964])
	#chromosomes.best = np.array([-0.1221, 0.825, -0.2782, -2.8425, -4.473])
	#chromosomes.best = np.array([-0.8905, 0.9556, -0.1357985, -7.47055, -2.9102])
	chromosomes.best = np.array([-0.1221, 0.825, -.1007, -2.8425, -1.0492])
	#chromosomes.best = np.array([-0.1221, 0.95212399, -0.1007, -3.59053283, -1.0492])
	
	
	# High yaw case
	#chromosomes.best = np.array([ 1.10179206, -0.10262596, -0.30135633,  5.00553137,  4.6864815 ])
	#chromosomes.best = np.array([ 0.5, -0.5,  0.5,  5, 5])

	chromosomes.secondbest = chromosomes.best



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
	#Weight1 - Tractor Yaw
	#Weight2 - Trailer Yaw
	#Weight3 - Kingpin X Pos
	
	#Weight4 - Z1
	#Weight5 - Z2
	
	
	# Because I am keeping the best result from the last set, I'm at risk of reaching a local max not a global max. Will test and see if it's good enough regardless
	chromosomes.weights[0] = chromosomes.best
	
	for x in range(1, 10):
		mut = mutation()
		if mut == 2:
			w1 = chromosomes.best[0]
		elif mut == 1:
		    w1 = chromosomes.secondbest[0]
		else:
			w1 = chromosomes.best[0] + random.uniform(-.5, .5)
		
		if w1 > 2:
			w1 = 2
		elif w1 < -2:
			w1 = -2
		
		
		mut = mutation()
		if mut == 2:
			w2 = chromosomes.best[1]
		elif mut == 1:
		    w2 = chromosomes.secondbest[1]
		else:
			w2 = chromosomes.best[1] + random.uniform(-.5, .5)
		if w2 > 2:
			w2 = 2
		elif w2 < -2:
			w2 = -2
		
		mut = mutation()
		if mut == 2:
			w3 = chromosomes.best[2]
		elif mut == 1:
			w3 = chromosomes.secondbest[2]
		else:
			w3 = chromosomes.best[2] + random.uniform(-.1, .1)
		if w3 > 1:
			w3 = 1
		elif w3 < -1:
			w3 = -1
		
		mut = mutation()
		if mut == 2:
			w4 = chromosomes.best[3]
		elif mut == 1:
			w4 = chromosomes.secondbest[3]
		else:
			w4 = chromosomes.best[3] + random.uniform(-1, 1)
		if w4 > 10:
			w4 = 10
		elif w4 < -10:
			w4 = -10
		
		
		mut = mutation()
		if mut == 2:
			w5 = chromosomes.best[4]
		elif mut == 1:
			w5 = chromosomes.secondbest[4]
		else:
			w5 = chromosomes.best[4] + random.uniform(-1, 1)
		if w5 > 10:
			w5 = 10
		elif w5 < -10:
			w5 = -10
		
		chromosomes.weights[x][0] = w1
		chromosomes.weights[x][1] = w2
		chromosomes.weights[x][2] = w3
		chromosomes.weights[x][3] = w4
		chromosomes.weights[x][4] = w5
		
	print(chromosomes.weights)

def calculate_theta(x):

	# Make variables for the weights for simplicity
	w1 = chromosomes.weights[x][0]
	w2 = chromosomes.weights[x][1]
	w3 = chromosomes.weights[x][2]
	w4 = chromosomes.weights[x][3]
	w5 = chromosomes.weights[x][4]
	
	# Neurons
	z1 = np.tanh(tractor.yaw * w1 + trailer.yaw * w2)
	z2 = np.tanh(trailer.x * w3)
	#print(str(z1) + ' : ' + str(z2))
	z3 = np.tanh(z1*w4 + z2*w5)
	
	return(z3)
	
def fitness(x,n):
	# Convert Yaw values to degrees so we can safely square
	tractoryaw = tractor.yaw * 180 / 3.14
	traileryaw = trailer.yaw * 180 / 3.14
	#fit = 1/((tractoryaw**2) + (traileryaw**2) + (trailer.y**2) + 10 * (trailer.x**2) + 0.001*n + 1)
	#fit = 1/((tractoryaw**2) + (traileryaw**2) + (trailer.x**2) + 0.001*n + 1)
	
	#fit = 1/((tractoryaw**2) + (traileryaw**2) + 10 * (trailer.x**2) + 0.001*n + 1)
	fit = 1/((tractoryaw**2) + (traileryaw**2) + 10 * (trailer.x**2) + 0.01*n + 1)
	if fit > chromosomes.fitness[x][0]:
		chromosomes.fitness[x][0] = fit
	
	chromosomes.fitness[x][0] = fit
	return(fit)
	
def findsecondfit():
	test = 0
	smaller = 0
	
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
# 						                    Simulation Stuff
#-------------------------------------------------------------------------------------------------------------------

def reset_pos(scenario):
	
	pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
	
	if scenario == 0:
		y = -50
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
	chromosomes_init()
	generations = 0
	numsteps = 500
	
	
	while not rospy.is_shutdown():
		
		# Update the pose of the links
		ttlinks = TractorTrailerLinks()
		ttlinks.show_gazebo_links()
		
		# Reset position of tractor trailer just in case
		reset_pos(0)
		
		# Generate Generation
		create_chromosomes()
		generations = generations + 1
		print("Generation: " + str(generations))
		
		# Open Files
		weights_file = open("Weights_Trailer.txt", "a")
		fitness_file = open("Fitness_Trailer.txt", "a")
		
		# Run Simulation
		for x in range(0, 10):
			print("Chromosome " + str(x))
			#for y in range(0, 5):
			for y in range(1, 2):
				# Decide length of test based on test case
				if y == 0:
					numsteps = 300
				elif (y == 1) or (y == 2):
					numsteps = 400
				else:
					numsteps = 600
					
				numsteps = 500
						
				reset_pos(y)
				for n in range(1, numsteps):
					
					# Update the pose of the links
					ttlinks = TractorTrailerLinks()
					ttlinks.show_gazebo_links()
					
					# Desired command
					speed, vel = calculate_speed(speed)
					theta = calculate_theta(x)
					
					# Publish movement command and wait for next loop
					driv_wheel_pub.publish(speed)
					pass_wheel_pub.publish(speed)
					steering_pub.publish(theta)
					#print(theta*180/3.14)
					rate.sleep()
					
					# Calculate fitness value
					fit = fitness(x, n)
					chromosomes.fitness[x][y] = fit
				
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
