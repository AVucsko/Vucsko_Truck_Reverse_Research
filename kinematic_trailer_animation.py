#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import time
import math


# --------------------------------------------------------
# ------------       Define Classes       ----------------
# --------------------------------------------------------
class tractor:
    def __init__(self, x, y, x_prev, y_prev, yaw, yaw_prev, vel):
        self.x = x
        self.y = y
        self.x_prev = x_prev
        self.y_prev = y_prev
        self.yaw = yaw
        self.yaw_prev = yaw_prev
        self.vel = vel
		
class trailer:
    def __init__(self, x, y, x_prev, y_prev, yaw, yaw_prev, vel):
        self.x = x
        self.y = y
        self.x_prev = x_prev
        self.y_prev = y_prev
        self.yaw = yaw
        self.yaw_prev = yaw_prev
        self.vel = vel

class pid:
    def __init__(self, integ, deriv, error, error_prev):
        self.integ = integ
        self.deriv = deriv
        self.error = error
        self.error_prev = error_prev


class sm:
    def __init__(self, s_x, s_x_prev, sy, sy_prev, error_x, error_x_dot, error_y, error_y_dot):
        self.s_x = s_x
        self.s_x_prev = s_x_prev
        self.s_y = s_y
        self.s_y_prev = s_y_prev
        self.error_x = error_x
        self.error_x_dot = error_x_dot
        self.error_y = error_y
        self.error_y_dot = error_y_dot

class nn:
    def __init__(self, w1, w2, w3, w4, w5, prev_scen):
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3
        self.w4 = w4
        self.w5 = w5
        self.prev_scen = prev_scen
    def decide_controller():
    	# scenario == 0: Fine Controller
    	# scenario == 1: HighX Controller
    	# scenario == 2: HighYaw Controller
    	
        if nn.prev_scen == 2:
            if abs(tractor.x) <= 5 and abs(trailer.yaw) <= 0.2:
                scenario = 0
            elif (abs(trailer.yaw) <= 0.2) and (abs(trailer.yaw) <= 0.2):
                scenario = 1
            else:
                scenario = 2
        if nn.prev_scen == 1:
            if abs(tractor.x) <= 5 and abs(trailer.yaw) <= 0.2:
                scenario = 0
            elif abs(trailer.yaw) <= 1.4:
                scenario = 1
            else:
                scenario = 2
        else:
            if abs(tractor.x) <= 5 and abs(trailer.yaw) <= 0.2:
        	    scenario = 0
            elif abs(trailer.yaw) <= 0.6:
                scenario = 1
            else:
                scenario = 2

        if scenario == 1:
            nn.w1, nn.w2, nn.w3, nn.w4, nn.w5 = -0.23021679, 1.08676092, -0.09843739, -3.9608538, -1.01370531
        elif scenario == 2:
            nn.w1, nn.w2, nn.w3, nn.w4, nn.w5 = -0.13244344, 0.61450203, -0.06354845, -3.70897267, -1.17183854
        else:
            nn.w1, nn.w2, nn.w3, nn.w4, nn.w5 = 1.10179206, -0.10262596, -0.30135633, 5.00553137, 4.6864815
    	
        nn.prev_scen = scenario

# --------------------------------------------------------
# ------------    Supporting Functions    ----------------
# --------------------------------------------------------

def rate_limit(delta, delta_prev, dt):
    # Currently limiting the steering command to 360 deg/s -- In line with rereax limiters at this speed
    if abs((abs(delta) - abs(delta_prev)))/dt >= 0.0628:
        if delta > delta_prev:
            delta = delta_prev + 0.0628
        else:
            delta = delta_prev - 0.0628
   
    return(delta)


# --------------------------------------------------------
# -----------------    Controllers    --------------------
# --------------------------------------------------------

def pid_controller(goal, dt):
    # PID Controller to Reach Desired Articulation Angle
    #P = -7
    #I = -3
    #D = -0.5
    P = -5
    I = 0
    D = 0
    
    # Define Errors
    pid.error = goal - (tractor.yaw - trailer.yaw)
    pid.integ = pid.integ + pid.error * dt
    pid.deriv = (pid.error - pid.error_prev) / dt
    
    # Desired Steering Angle
    delta = pid.error * P + pid.integ * I + pid.deriv * D
    
    if delta < -0.52:
        delta = -0.52
    elif delta > 0.52:
        delta = 0.52

    pid.error_prev = pid.error
    return(delta)
    
    
    
def sm_controller(dt):
    # Controller Weights

    # Please use these weights
    k1x, qx = 0.1, -0.20
    k1y, qy = -0.4, -0.2

    # Set 2
    lat_gain = 1
    yaw_gain = 0.55
    k1x, qx = 0.4*lat_gain, 0.15*lat_gain
    k1y, qy = -0.4*yaw_gain, -0.4*yaw_gain


    # I think we need to set this up so we have a sliding surface for lat error
    # and a sliding surface for yaw error
    # Then we have the PID articulation angle thing on top
    # Maybe that could work?
    
    # Sliding surface - Lat Error
    sm.error_x = trailer.x
    sm.error_x_dot = (trailer.x-trailer.x_prev)/dt
    sm.s_x = sm.error_x_dot + k1x*sm.error_x
    sm.s_dot_x = qy*(sm.s_x-sm.s_x_prev)/dt

    # Sliding surface - Trailer Angle
    sm.error_y = trailer.yaw
    sm.error_y_dot = (trailer.yaw-trailer.yaw_prev)/dt
    sm.s_y = sm.error_y_dot + k1y*sm.error_y
    sm.s_dot_y = qy*(sm.s_y-sm.s_y_prev)/dt

    
    if sm.s_x < 0:
        sm.s_dot_x = qx
    ##    #c = L2*q*-1/(v**2*l*math.cos(psi)*math.cos(error_theta))
    else:
        sm.s_dot_x = -qx
    ##    #c = L2*q*1/(v**2*l*math.cos(psi)*math.cos(error_theta))
##
    if sm.s_y < 0:
        sm.s_dot_y = qy
    else:
        sm.s_dot_y = -qy
        
    # Trying to move between modes
    #if abs(error) < 1:
    #    s_x = -s_x
    #else:
    #    s_x = s_x
    
    theta_x = -sm.s_x + -sm.s_dot_x
    theta_y = -sm.s_y + -sm.s_dot_y
    theta = theta_x + theta_y
    sm.s_x_prev = sm.s_x
    sm.s_y_prev = sm.s_y

    # Can we put another PID controller in to keep it close to zero error?
    # Should we even need another PID controller or should the sliding mode handle it?
    # Can I have variable gains?
    if np.abs(tractor.x) < 0.2:
        theta = trailer.yaw
        
    # Limit the articulation angle we can request
    if theta > 1.57:
        theta = 1.57
    elif theta < -1.57:
        theta = -1.57
        
        
    return(theta)


def nn_controller():
    # Decide which controller to use
    nn.decide_controller()
	
	# Neurons
    z1 = np.tanh(tractor.yaw * nn.w1 + trailer.yaw * nn.w2)
    z2 = np.tanh(tractor.x * nn.w3)
    z3 = np.tanh(z1*nn.w4 + z2*nn.w5)
	
    return(z3)


# --------------------------------------------------------
# -----------------  Kinematic Model  --------------------
# --------------------------------------------------------

def kin_model(delta, dt, vel):
    # Important to keep in mind x and y are flipped from normal

    # x - X error measured at kingpin (meters)
    # y - Y error measured at kingpin (meters)
    # psi_tract - Angle of tractor measured relative to y-axis (rad)
    # psi_trail - Angle of trailer measured relative to y-axis (rad)
    # delta - Steered wheel angle (rad)
    # dt - Time step (s)
    # vel = Tractor velocity (m/s)
    
    # Do the math measured at the rear axle / kingpin - L1c = 0
    # Setup vehicle parameters
    L1c = 0 #meters
    L_tract = 2 #meters
    L_trail = 16.15 #meters
    art_angle = tractor.yaw - trailer.yaw
    if art_angle > 1.57:
        art_angle = 1.57
    elif art_angle < -1.57:
        art_angle = -1.57
    
    # Kinematic model
    x_dot = vel*np.sin(tractor.yaw)
    y_dot = vel*np.cos(tractor.yaw)
    psi_tract_dot = vel*np.tan(delta) / L_tract
    psi_trail_dot = vel*np.sin(art_angle)/L_trail + vel*L1c*np.cos(art_angle)/(L_tract*L_trail)*np.tan(delta)
    
    # Update positions
    tractor.x_prev = tractor.x
    tractor.y_prev = tractor.y
    trailer.x_prev = trailer.x
    trailer.y_prev = trailer.y
    trailer.yaw_prev = trailer.yaw
    
    tractor.x = tractor.x + x_dot * dt
    tractor.y = tractor.y + y_dot * dt
    tractor.yaw = tractor.yaw + psi_tract_dot * dt
    trailer.yaw = trailer.yaw + psi_trail_dot * dt
    art_angle = tractor.yaw - trailer.yaw
    trailer.x = tractor.x - np.sin(trailer.yaw)*L_trail
    trailer.y = tractor.y - np.cos(trailer.yaw)*L_trail
    #print(x_dot, dt, x_new)
    
    # Limiters
    #if np.abs(art_angle) > 1.57:
    #    print('Jackknife!')
    #    psi_trail = psi_tract - 1.57
    
    # Return
    return()
    
    
    
# --------------------------------------------------------
# -----------------        Main       --------------------
# --------------------------------------------------------
def main():
    # Setup Simulation
    controller_choice = 'sm'
    maneuver_choice = 'offset'
    save2file = 0
    sim_time = 100 #seconds

    if maneuver_choice == 'offset':
        # For offsetback
        dt = 0.1 #seconds
        steps_max = int(np.floor(sim_time / dt)) #iterations
        tractor.x = 5 # Error measured at kingpin (meters)
        tractor.y = 55 # Error measured at kingpin (meters)
        trailer.x = tractor.x
        trailer.y = 55 - 16.15
        tractor.yaw = 0 # Error measured relative to y axis (rad)
        trailer.yaw = 0 # Error measured relative to y axis (rad)
        trailer.yaw_prev = trailer.yaw
        vel = -1 #m/s

        if controller_choice == 'nn':
            filename = 'nn_offset.csv'
        elif controller_choice == 'sm':
            filename = 'sm_offset.csv'
            
    elif maneuver_choice == 'alley':
        # For alleypark
        dt = 0.1 #seconds
        steps_max = int(np.floor(sim_time / dt)) #iterations
        tractor.x = 20 # Error measured at kingpin (meters)
        tractor.y = 5 # Error measured at kingpin (meters)
        trailer.x = tractor.x
        trailer.y = 5 - 16.15
        tractor.yaw = 3.14/2 # Error measured relative to y axis (rad)
        trailer.yaw = 3.14/2 # Error measured relative to y axis (rad)
        trailer.yaw_prev = trailer.yaw
        vel = -1 #m/s

        if controller_choice == 'nn':
            filename = 'nn_offset.csv'
        elif controller_choice == 'sm':
            filename = 'sm_offset.csv'
    
    # Initialize variables
    delta = 0 # Steered wheel angle (rad)
    time = 0
    pid.integ = 0
    pid.deriv = 0
    pid.error = 0
    pid.error_prev = 0
    delta_prev = 0
    tractor.x_prev = tractor.x
    trailer.x_prev = trailer.x
    error_prev = 0
    sm.s_x = 0
    sm.s_y = 0
    sm.s_x_prev = 0
    sm.s_y_prev = 0
    simulation_variables = np.zeros([steps_max, 8])
    nn.prev_scen = 0
    
    # Setup Goal Array
    goal_array = np.ones([steps_max, 1])
    
    
    # Run simulation
    with open(filename,'a') as f:
        for idx in range(0, steps_max):

            if controller_choice == 'nn':
                # Call NN Controller to get Steering Angle
                delta = nn_controller()
            else:
                # Call SM Controller to get Target Art. Angle
                goal_array[idx] = sm_controller(dt)
                
                # Call PID Controller to reach Target Art. Angle
                delta = pid_controller(goal_array[idx], dt)
            
            # Rate Limit Steering Command
            delta = rate_limit(delta, delta_prev, dt)
    
            # Iterate positions
            delta_prev = delta
            kin_model(delta, dt, vel)
            
            # Save data
            simulation_variables[idx,:] = np.array([tractor.x, tractor.y, trailer.x, trailer.y, tractor.yaw, trailer.yaw, delta, time], dtype=object)
            if save2file == 1:
                np.savetxt(f,[simulation_variables[idx,:]],delimiter=",",fmt="%f")
            time = time + dt
    
    # Display results 
    # Create Plot
    fig, ax = plt.subplots(1, 1, figsize=(6,6))

    # Define steps required for animation
    def animate(i):
        ax.cla()
        trail_pat = plt.Rectangle(xy=(simulation_variables[0,2] - 1*math.cos(simulation_variables[0,5]), simulation_variables[0,3] + 1*math.sin(simulation_variables[0,5])), height=16.15, width=2, angle=simulation_variables[0,5],color='r')
        trail_pat.rotation_point='center'
        trail_pat.angle = simulation_variables[0,5] * -180/3.14
        tract_pat = plt.Rectangle(xy=(simulation_variables[0,0] - 1*math.cos(simulation_variables[0,4]), simulation_variables[0,1] + 1*math.sin(simulation_variables[0,4])), width=2, height=2, angle = simulation_variables[0,4], color='k')
        tract_pat.rotation_point='center'
        tract_pat.angle = simulation_variables[0,4] * -180/3.14
        ax.add_patch(trail_pat)
        ax.add_patch(tract_pat)
        
        trail_pat = plt.Rectangle(xy=(simulation_variables[i,2] - 1*math.cos(simulation_variables[i,5]), simulation_variables[i,3] + 1*math.sin(simulation_variables[i,5])), height=16.15, width=2, angle=simulation_variables[i,5],color='r')
        trail_pat.rotation_point='center'
        trail_pat.angle = simulation_variables[i,5] * -180/3.14
        tract_pat = plt.Rectangle(xy=(simulation_variables[i,0] - 1*math.cos(simulation_variables[i,4]), simulation_variables[i,1] + 1*math.sin(simulation_variables[i,4])), width=2, height=2, angle = simulation_variables[i,4], color='k')
        tract_pat.rotation_point='center'
        tract_pat.angle = simulation_variables[i,4] * -180/3.14
        ax.add_patch(trail_pat)
        ax.add_patch(tract_pat)
        plt.plot(simulation_variables[:i,2], simulation_variables[:i,3], color='r')
        plt.plot(simulation_variables[:i,0], simulation_variables[:i,1], color='k')
        ax.set_xlim([-70, 70])
        ax.set_ylim([-70, 70])

    # Create the animation    
    anim = animation.FuncAnimation(fig, animate, frames=steps_max, interval=1, blit=False)
    #plt.xlim([-70,70])
    #plt.ylim([-10,70])
    plt.plot(simulation_variables[:,7], simulation_variables[:,4]-simulation_variables[:,5],label="Observed Angle")
    plt.plot(simulation_variables[:,7], simulation_variables[:,7]-simulation_variables[:,7]+1.74,'r',label="Jackknife Boundary")
    plt.plot(simulation_variables[:,7], simulation_variables[:,7]-simulation_variables[:,7]-1.74,'r')
    plt.title('Offset Back Articulation Angle')
    plt.xlabel('Simulation Time (sec)')
    plt.ylabel('Articulation Angle (rad)')
    plt.legend(loc="upper right")
    plt.show()
    


main()
