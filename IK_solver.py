# -*- coding: utf-8 -*-
"""
Created on Sat Jun  5 13:50:10 2021

@author: Michael
"""

import sys
import time
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
import math
import numpy as np
import copy
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import time


time.sleep(1)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
# angle offset and sign for each servo to be consistent with simulation
angle_offset = [0, -90, 0, 0, 0, 0, 0]
sign_flip = [1, -1, 1, -1, 1, 1, 1,] 

servo_name = [6, 5, 4, 3, 2, 1, 0]
def servo_set_angle(servo, angle, speed=10):
    pulse = 500 + int(sign_flip[servo]*375/90*angle+375/90*angle_offset[servo])
    Board.setBusServoPulse(servo_name[servo], pulse, speed)

servo_set_angle(6, 0,300)
servo_set_angle(5, 0, 300)
servo_set_angle(4, 0, 300)
servo_set_angle(3, 0, 300)
servo_set_angle(2, 0, 300)
servo_set_angle(1, 0, 300)
servo_set_angle(0, 0, 300)
time.sleep(10)


mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()
ax =  fig.gca(projection='3d')
    
arm_plot = ax.plot([],[],[], 'o-', mfc='red')#,label='arm')
endeffector_plot = ax.plot([],[],[], 'o', mfc='green')  
target_plot = ax.plot([],[],[],'o', mfc='blue')

# link class, contains information for joint and rod attached to it.
 
class Link():
    
    def __init__(self, link_vector, link_index, angles = np.zeros(3), angle_limits = np.zeros((3,2))):
        
        self.length = length = np.linalg.norm(link_vector)
        self.link_index = link_index
        self.link_vector = link_vector
        self.angles = np.array(angles, dtype=np.float32)
        self.angle_limits = np.array(angle_limits, dtype=np.float32)
        
    def Rx_matrix(self):
        
        matrix = np.array([[1, 0, 0],[0, np.cos(self.angles[0]), -1*np.sin(self.angles[0])],[0, np.sin(self.angles[0]), np.cos(self.angles[0])]])               
        return matrix
        
    def Ry_matrix(self):
        
        matrix = np.array([[np.cos(self.angles[1]), 0, np.sin(self.angles[1])],[0, 1, 0], [-np.sin(self.angles[1]), 0, np.cos(self.angles[1])]])
        return matrix
        
    def Rz_matrix(self):
        
        matrix = np.array([[np.cos(self.angles[2]), -1*np.sin(self.angles[2]), 0],[np.sin(self.angles[2]), np.cos(self.angles[2]), 0], [0, 0, 1]])
        return matrix 
    
    def rotation_matrix(self):
        
        matrix = np.matmul(self.Rz_matrix(), np.matmul(self.Ry_matrix(),self.Rx_matrix()))
        return matrix
    
    def set_angle(self, angle, angle_index = None):
        
        if angle_index == None:
            
            self.angles[0] = np.clip(angle[0], self.angle_limits[0, 0], self.angle_limits[0,1])
            self.angles[1] = np.clip(angle[1], self.angle_limits[1, 0], self.angle_limits[1,1]) 
            self.angles[2] = np.clip(angle[2], self.angle_limits[2, 0], self.angle_limits[2,1])
        
        else:
            self.angles[angle_index] = np.clip(angle, self.angle_limits[angle_index, 0], self.angle_limits[angle_index,1])
        return self.angles
        
    def get_angle(self, angle_index = None):
        
        if angle_index == None: 
            return self.angles
        else:
            return self.angles[angle_index]
    
    
# perform gradient descent on arm parameters

class Arm():
    
    def __init__(self, link_list, base_location = np.array([0,0,0],dtype=np.float32)):
        self.link_list = link_list
        self.base_location = np.array(base_location, dtype=np.float32) 
        
    
    def get_joint_locations(self):

        # asemble arm into coordinates with respect to the base location
        
        rotated_locations = [np.zeros(3)]*(len(self.link_list))
        
        
        for rotation_index in range(len(rotated_locations),-1,-1):
            for link_index in range(rotation_index, len(rotated_locations),1):
                rotated_locations[link_index] = np.matmul(self.link_list[rotation_index].rotation_matrix(), rotated_locations[link_index] + self.link_list[rotation_index].link_vector)
                
        
        for link_index in range(0, len(rotated_locations),1):
            rotated_locations[link_index] = rotated_locations[link_index] + self.base_location
        return rotated_locations
        
    
    def get_arm_angles(self):
        angle_list = []
        for link in self.link_list:
            angle_list.append(link.get_angle())
        return angle_list
    
    
    def get_endeffector_location(self):
        
        # return the location of the last links end effector
        locations = self.get_joint_locations()
        return locations[-1]
            
            
    def get_link_angle(self, link_index, angle_index):
        
         if angle_index == None:
            angle = self.link_list[link_index].get_angle()
         else:
            angle = self.link_list[link_index].get_angle(angle_index)

         return angle
        
        
    def set_link_angle(self, link_index, angle, angle_index = None):
        
        if angle_index == None:
            self.link_list[link_index].set_angle(angle)
        else:
            self.link_list[link_index].set_angle(angle, angle_index)
            


    def track_trajectory(self, path, run_time, delta_t = .2):
        path = path
        run_time = run_time
        arm_angle_history = []
        error_history = []
        end_effector_path = []
        target_path = []
        
        last_state = None
        next_state = None
        
        t = 0
        
        old_time = time.time()
        while t < run_time:
            print("target path " + str(path(t)))
            print("arm location " + str(self.get_endeffector_location()))
            error = self.cyclic_gradient_descent(path(t)) #self.delta_search(path(t)) #self.gradient_descent(path(t)) #self.cyclic_gradient_descent(path(t))  #  # self.delta_search(path(t)) self.gradient_descent(path(t)) #self.delta_search(path(t))# #self.delta_search(path(t))  #self.delta_search(path(t)) #self.gradient_descent(path(t))
            print("error for time " + str(t) + " is "+ str(error))
            arm_angle_history.append([t, self.get_arm_angles()])
            end_effector_path.append([t, self.get_endeffector_location()])
            target_path.append([t, path(t)])
            error_history.append([t,error])
            t += delta_t
            plot_arm(self.get_joint_locations(), path(t))
            self.update_arm()
            new_time = time.time()
            time_past = new_time - old_time
            if time_past < delta_t:
                time.sleep(delta_t - time_past)

        return arm_angle_history, end_effector_path, target_path, error_history
    
    # update physical arm
    def update_arm(self):
        
        # associate servos to rotations for each joint
        
        servo_dictionary ={}
        servo_dictionary[(0,2)] = 0
        servo_dictionary[(1,1)] = 1
        servo_dictionary[(2,1)] = 2
        servo_dictionary[(3,1)] = 3

        for link_index in range(len(self.link_list)):
            for angle_index in range(3):
                if (link_index, angle_index) in servo_dictionary:
                    angle = self.link_list[link_index].get_angle(angle_index)
                    servo_set_angle(link_index, 360/(2*math.pi)*angle, 300)
    
    def delta_search(self, target_position, delta_search = np.array([-.1,-.05,-.02,-.01, 0, .01, .02, .05, .1]), cyclic_iterations = 20, epsilon = .01):
    
    
        buffer_arm = copy.deepcopy(self)
        base_link_angle = math.atan2(target_position[1], target_position[0])
        buffer_arm.link_list[0].set_angle(base_link_angle,2)
        L_best = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
        
        delta_best = 0
        
        for iteration in range(cyclic_iterations):
  
            for link_index in range(1,len(buffer_arm.link_list),1):
                for angle_index in range(3):
                    
                    # check if it's possible to move joint 
                    if buffer_arm.link_list[link_index].angle_limits[angle_index,0] != buffer_arm.link_list[link_index].angle_limits[angle_index,1]:   
                        angle = buffer_arm.link_list[link_index].get_angle(angle_index)
                    
                        for delta in delta_search:

                            buffer_arm.link_list[link_index].set_angle(angle + delta, angle_index)
               
                            # get perturbed error 
                            L_delta = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                            
                            if L_delta <= L_best:
                                delta_best = delta
                                L_best =  L_delta 
                                
                        buffer_arm.link_list[link_index].set_angle(angle + delta_best, angle_index)
                        
                        if L_best < epsilon:
                            self.link_list = buffer_arm.link_list
                            return L_best
        self.link_list = buffer_arm.link_list
                    
        return L_best
                                
          

    # numerically compute jacobian with respect to l2 norm loss for parameters, apply
    # gradient descent to update parameters  

    def gradient_descent(self, target_position, learning_rate = .01, perturbation = .001, epsilon = .1, gradient_descent_iteration = 5):
        
        buffer_arm = copy.deepcopy(self)
        base_link_angle = math.atan2(target_position[1], target_position[0])
        buffer_arm.link_list[0].set_angle(base_link_angle,2)
        
        
        # apply gradient descent until convergence or iteration number reached.     
        
        for iteration in range(gradient_descent_iteration):
            
            jacobian_list = []
            
            for link_index in range(0,len(buffer_arm.link_list),1):
                
                # list which holds partial derivatives for each entry for link
                link_jacobian = []
                
                for angle_index in range(3):
                    
                    # check if it's possible to move joint
                    if buffer_arm.link_list[link_index].angle_limits[angle_index,0] != buffer_arm.link_list[link_index].angle_limits[angle_index,1]:
                       
                    
                        # return angle
                        angle = buffer_arm.link_list[link_index].get_angle(angle_index)
                        
                        #perturb angle positive
                    
                        buffer_arm.link_list[link_index].set_angle(angle + perturbation, angle_index)
       
                        # get perturbed error 
                        L_theta_perturb_positive = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                        
                        # perturb angle negative
                        
                        buffer_arm.link_list[link_index].set_angle(angle - perturbation, angle_index)
       
                        L_theta_perturb_negative = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                        
                        # approximate derivative
                                                         
                        dL_dtheta = (L_theta_perturb_positive - L_theta_perturb_negative)/(2*perturbation) 
                        
                        # restore angle
                        buffer_arm.link_list[link_index].set_angle(angle, angle_index)
                        
                        # enter jacobian information
                        link_jacobian.append(dL_dtheta)   
                    else:
                        link_jacobian.append(0)
                        
        
                jacobian_list.append(link_jacobian)
                
        
            # curent error
            error = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
            
            # best link_list is the current one in buffer
            best_link_list = copy.deepcopy(buffer_arm.link_list)
            base_link_list = copy.deepcopy(buffer_arm.link_list)
            
            powers =[-1, -.5, 0, .5, 1]
            
            for power in powers:
                
                buffer_arm.link_list = copy.deepcopy(base_link_list)       
                scale = 1/(10**power)
                

                for link_index in range(1,len(buffer_arm.link_list),1):
                    for angle_index in range(3):
                        angle = buffer_arm.get_link_angle(link_index, angle_index)
                        angle += -1*scale*learning_rate*jacobian_list[link_index][angle_index]
                        buffer_arm.link_list[link_index].set_angle(angle, angle_index)
            
                new_error = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                
                
                if new_error <= error:
                    best_link_list = copy.deepcopy(buffer_arm.link_list)
                    error = new_error
                    
                
            buffer_arm.link_list = copy.deepcopy(best_link_list)
            
            # if good error is reached, return
            
            if error < epsilon:
                self.link_list = copy.deepcopy(buffer_arm.link_list)
                return error
            
        # else search to exhaustion and return         
            
        self.link_list = copy.deepcopy(buffer_arm.link_list)
        return error
    
  
    # applies gradient descent on each joint in a cyclic fashion.
    
    
    def cyclic_gradient_descent(self, target_position, learning_rate = .01, perturbation = .001, epsilon = .01, gradient_descent_iteration = 5):

        
         buffer_arm = copy.deepcopy(self)
         base_link_angle = math.atan2(target_position[1], target_position[0])
         buffer_arm.link_list[0].set_angle(base_link_angle,2)
        
        # apply gradient descent until convergence or iteration number reached.            
        
         for iteration in range(gradient_descent_iteration):              
            
            for link_index in range(1,len(buffer_arm.link_list),1):
                
                # list which holds partial derivatives for each entry for link
                link_jacobian = []
                
                for angle_index in range(3):
                    
                    # check if it's possible to move joint
                    if buffer_arm.link_list[link_index].angle_limits[angle_index,0] != buffer_arm.link_list[link_index].angle_limits[angle_index,1]:
                    
                        # return angle
                        angle = buffer_arm.link_list[link_index].get_angle(angle_index)
                        
                        #perturb angle positive
                    
                        buffer_arm.link_list[link_index].set_angle(angle + perturbation, angle_index)
       
                        # get perturbed error 
                        L_theta_perturb_positive = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                        
                        # perturb angle negative
                        
                        buffer_arm.link_list[link_index].set_angle(angle - perturbation, angle_index)
       
                        L_theta_perturb_negative = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                        
                        # approximate derivative
                                                         
                        dL_dtheta = (L_theta_perturb_positive - L_theta_perturb_negative)/(2*perturbation) 
                        
                        # restore angle
                        buffer_arm.link_list[link_index].set_angle(angle, angle_index)
                        
                        # enter jacobian information
                        link_jacobian.append(dL_dtheta)   
                    else:
                        link_jacobian.append(0)
                        

                # curent error
                error = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                
                # best link_list is the current one in buffer
                best_link_list = copy.deepcopy(buffer_arm.link_list)
                base_link_list = copy.deepcopy(buffer_arm.link_list)
                
                powers =[-1, -.5, 0, .5, 1]
                
                for power in powers:
                    
                    buffer_arm.link_list = copy.deepcopy(base_link_list)      
                    scale = 1/(10**power)

                    for angle_index in range(3):
                         angle = buffer_arm.get_link_angle(link_index, angle_index)
                         angle += -1*scale*learning_rate*link_jacobian[angle_index]
                         buffer_arm.link_list[link_index].set_angle(angle, angle_index)
                
                    new_error = np.linalg.norm(buffer_arm.get_endeffector_location() - target_position)
                    
                    if new_error <= error:
                        best_link_list = copy.deepcopy(buffer_arm.link_list)
                        error = new_error
                    
                buffer_arm.link_list = copy.deepcopy(best_link_list)
                
                if error < epsilon:
                    self.link_list = copy.deepcopy(buffer_arm.link_list)
                    return error
            
        # else search to exhaustion and return         
            
         self.link_list = copy.deepcopy(buffer_arm.link_list)
         return error
 
def plot_arm(locations, target):
    
  
    # generate arm plot
    
    arm = np.zeros((3,len(locations)+1))
    
    for index in range(len(locations)):
        arm[:,index+1] = locations[index]
                
                
    arm_plot[0].set_data(arm[0,:], arm[1,:])
    arm_plot[0].set_3d_properties(arm[2,:]) #, 'o-', mfc='red')#,label='arm')
    endeffector_plot[0].set_data(arm[0,len(locations)], arm[1,len(locations)])
    endeffector_plot[0].set_3d_properties(arm[2,len(locations)]) #, 'o', mfc='green',label='end')  
    target_plot[0].set_data(target[0],target[1])
    target_plot[0].set_3d_properties(target[2])
    
    
  
    ax.axes.set_xlim3d(-20,20)
    ax.axes.set_ylim3d(-20,20) 
    ax.axes.set_zlim3d(0,20) 
    
    plt.pause(.00000000001)
        

def main():
    
    base_link = Link(np.array([0,0,2.5]), 0, np.array([0,0,20]), np.array([[0,0], [0,0], [-math.pi/2, math.pi/2]]))
    link_1 = Link([4,0,0], 1, np.array([0, 0, 0]), np.array([[0,0], [-math.pi, 0],[0,0]]))
    link_2 = Link([3.75,0,0], 2, np.array([0, .5, 0]), np.array([[0,0], [.1, math.pi*3/4],[0,0]]))
    link_3 = Link([6.5,0,0], 3, np.array([0, .5, 0]), np.array([[0,0], [.1, math.pi*3/4],[0,0]]))

    
    links = [base_link, link_1, link_2, link_3] 
    
    

    #path = lambda t: [5*np.sin(2*math.pi*t/10), 5*np.cos(2*math.pi*t/10), 0] 
    
    #path = lambda t: (1-t/10)*np.array([6, 0, 2], dtype=np.float32) + 0/10*np.array([2,2,3], dtype=np.float32)   
    path = lambda t:np.array([2*math.cos(2*math.pi*t/5),2*math.sin(2*math.pi*t/5), math.sin(2*math.pi*t/10)]) + np.array([5,0,3])
    
    arm = Arm(links)
    locations = arm.get_joint_locations()
    plot_arm(locations, np.array([0.,0.,0.]))
    print(arm.get_endeffector_location())
    
    arm_angle_history, end_effector_path, target_path, error_history = arm.track_trajectory(path, 20)
    
    
    arm_data = np.zeros((8, len(target_path)))
    for index in range(len(arm_angle_history)):
        arm_data[0:3, index] = end_effector_path[index][1]
        arm_data[3:6, index] = target_path[index][1]
        arm_data[6, index] = error_history[index][1]
        arm_data[7, index] = end_effector_path[index][0]
    
    
    
    plt.figure()
    plt.plot(arm_data[7,:], arm_data[0,:], label='arm')
    plt.plot(arm_data[7,:], arm_data[3,:], label='target')
    plt.title('x location')
    plt.xlabel("s")
    plt.ylabel("in")
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(arm_data[7,:], arm_data[1,:], label='arm')
    plt.plot(arm_data[7,:], arm_data[4,:], label='target')
    plt.title('y location')
    plt.xlabel("s")
    plt.ylabel("in")
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(arm_data[7,:], arm_data[2,:], label='arm')
    plt.plot(arm_data[7,:], arm_data[5,:], label='target')
    plt.title('z location')
    plt.xlabel("s")
    plt.ylabel("in")
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(arm_data[7,:], arm_data[6,:])
    plt.xlabel("s")
    plt.ylabel("in")
    plt.title("End Effector Error Distance vs Time")
    plt.show()
    time.sleep(10)

main()
