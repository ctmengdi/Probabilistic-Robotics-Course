#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
from probabilistic_lib.functions import angle_wrap, comp, state_inv, state_inv_jacobian, compInv
import scipy.linalg
import rospy
from numpy.linalg import inv

#============================================================================
class EKF_SLAM(object):
    '''
    Class to hold the whole EKF-SLAM.
    '''
    
    #========================================================================
    def __init__(self, x0,y0,theta0, odom_lin_sigma, 
                 odom_ang_sigma, meas_rng_noise, meas_ang_noise):
        '''
        Initializes the ekf filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        self.chi_thres = 0 # TODO chose your own value
       
        # Odometry uncertainty 
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
                            [ 0, self.odom_lin_sigma**2, 0 ],\
                            [ 0, 0, self.odom_ang_sigma**2]])
        
        # Measurements uncertainty
        self.Rk=np.eye(2)
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # State vector initialization
        self.xk = 1.0*np.array([x0,y0,theta0]) # Position
        self.Pk = np.zeros((3,3)) # Uncertainty
        
        # Initialize buffer for forcing observing n times a feature before 
        # adding it to the map
        self.featureObservedN = np.array([])
        self.min_observations = 5

        self.chi_thres = 0.103
    
    #========================================================================
    def get_number_of_features_in_map(self):
        '''
        returns the number of features in the map
        '''
        return (self.xk.size-3)/2
    
    #========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odometry [x y ang].
        Returns [range theta]
        '''
        # Line points
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        # Compute line (a, b, c) and range
        line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
        pt = np.array([odom[0], odom[1], 1])
        dist = np.dot(pt, line) / np.linalg.norm(line[:2])
        
        # Compute angle
        if dist < 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
        
    #========================================================================
    def predict(self, uk):
        
        '''
        Predicts the position of the robot according to the previous position and the odometry measurements. It also updates the uncertainty of the position
        '''
        #TODO: Program this function
        # - Update self.xk and self.Pk using uk and self.Qk
             
        # Compound robot with odometry
        xk = comp(self.xk[0:3], uk)
        # Compute jacobians of the composition with respect to robot (A_k) 
        # and odometry (W_k)
        Ak = np.array([[1, 0, -sin(self.xk[2])*uk[0]-cos(self.xk[2])*uk[1]], [0, 1, cos(self.xk[2])*uk[0]-sin(self.xk[2])*uk[1]], [0, 0, 1]])
        Wk = np.array([[cos(self.xk[2]), -sin(self.xk[2]), 0], [sin(self.xk[2]), cos(self.xk[2]), 0], [0, 0, 1]])
        # Prepare the F_k and G_k matrix for the new uncertainty computation
        num_fea = self.get_number_of_features_in_map()

        Fk = np.eye(3+2*num_fea)
        Fk[0:3, 0:3] = Ak
        Gk = np.zeros((3+2*num_fea, 3))
        Gk[0:3, 0:3] = Wk
        # Compute uncertainty
        Pk = np.dot(np.dot(Fk,self.Pk),np.transpose(Fk)) + np.dot(np.dot(Gk,self.Qk),np.transpose(Gk))
        # Update the class variables
        self.xk[0:3] = xk
        self.Pk = Pk
    #========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICCN for each feature of the scan.
        '''
    
        #TODO: Program this function
        # fore each sensed line do:
        #   1- Transform the sensed line to polar

        #   2- for each feature of the map (in the state vector) compute the 
        #      mahalanobis distance
        #   3- Data association
        
        # Init variable
        Innovk_List   = []
        H_k_List      = []
        Rk_List       = []
        Sk_list       = []
        idx_not_associated =[]

        # for each sensed line
        for i in range(0, lines.shape[0]):

            # transform the sensed line to polar
            z = self.get_polar_line(lines[i,:], np.array([0,0,0]))

            # Variables for finding minimum
            minD = 1e9
            minj = -1
            print(self.xk.shape)
            # for each feature in the map
            for j in range(3, self.xk.shape[0], 2):

                D,v,h,H= self.lineDist(z, j)

                # mahalanobis distance
                if np.sqrt(D) < minD:
                     minj = j
                     minz = z
                     minh = h
                     minH = H
                     minv = v
                     #minS = S
                     minD = D

            if minD < self.chi_thres:
                 #print("\t{:. 2f} -> {:.2f}".format(minz, minh))
                # Append results
                 H_k_List.append(minH)
                 Innovk_List.append(minv)
                 #Sk_list.append(minS)
                 Rk_List.append(self.Rk)
            else:
                print('Adding to idx not associated')
                idx_not_associated.append(i)

        print('idx_not_associated', idx_not_associated)
                
        return Innovk_List, H_k_List , Rk_List, np.array(idx_not_associated)
        
    #========================================================================
    def update_position(self, Innovk_List, H_k_List, Rk_List) :
        '''
        Updates the position of the robot according to the given the position
        and the data association parameters.
        Returns state vector and uncertainty.
        
        '''
        #TODO: Program this function
        if len(Innovk_List)<1:
            return

        # Compose list of matrices as single matrices
        n = len(H_k_List)
        H = np.zeros((2*n, self.xk.size))
        v = np.zeros((2*n))
        #S = np.zeros((2*n, 2*n))
        R = np.zeros((2*n, 2*n))
        for i in range(n):
            H[2*i:2*i+2, :] = H_k_List[i]
            v[2*i:2*i+2] = Innovk_List[i]
            #S[2*i:2*i+2, 2*i:2*i+2] = Sk_list[i]
            R[2*i:2*i+2, 2*i:2*i+2] = Rk_List[i]
        
        S = np.dot(np.dot(H,self.Pk), np.transpose(H)) + R
        # Kalman Gain
        I = np.eye(self.xk.size)
        K = np.dot(np.dot(self.Pk, np.transpose(H)), inv(S))

        # Update Position
        self.xk += np.squeeze(np.dot(K, v))

        # Update Uncertainty
        self.Pk = np.dot(np.dot((I - np.dot(K, H)), self.Pk), np.transpose(I - np.dot(K, H))) + np.dot(np.dot(K, R), np.transpose(K)) 

    #========================================================================
    def state_augmentation(self, lines, idx):
        '''
        given the whole set of lines read by the kineckt sensor and the
        indexes that have not been associated augment the state vector to 
        introduce the new features
        '''
        # If no features to add to the map exit function
        if idx.size<1:
            print('No lines to augment with')
            return

        num_fea = self.get_number_of_features_in_map()

        Fk_f = np.eye(3+2*num_fea)
        Gk_f = np.zeros((3+2*num_fea,2))

        # TODO Program this function
        for i in range(idx.size):
            polar_line = self.get_polar_line(lines[idx[i]],np.array([0,0,0]))
            z = self.tfPolarLine(self.xk[0:3],polar_line)

            self.xk = np.hstack((self.xk, z[0]))
            Fk_f = np.vstack((Fk_f, np.hstack((z[1],np.zeros((2,2*num_fea))))))
            Gk_f = np.vstack((Gk_f,z[2]))

        self.Pk = np.dot(np.dot(Fk_f,self.Pk),np.transpose(Fk_f)) + np.dot(np.dot(Gk_f,self.Rk),np.transpose(Gk_f))
            

    #========================================================================
    def tfPolarLine(self,tf,line):
        '''
        Transforms a polar line in the robot frame to a polar line in the
        world frame
        '''
        # Decompose transformation
        # Decompose transformation
        x_x = tf[0]
        x_y = tf[1]
        x_ang = tf[2]  
        
        # Compute the new phi
        phi = angle_wrap(line[1] + x_ang)
        
        rho_ = line[0] + x_x * np.cos(phi) + x_y * np.sin(phi)
        sign = 1
        if rho_ <0:
            rho_ = -rho_
            phi = angle_wrap(phi+pi)   
            sign = -1
        
        # Allocate jacobians
        H_tf = np.zeros((2,3))
        H_line = np.eye(2)

        # TODO: Evaluate jacobian respect to transformation
        H_tf = np.array([[cos(phi), sin(phi), -x_x*sin(phi)+x_y*sin(phi)],[0,0,1]])
        # TODO: Evaluate jacobian respect to line
        H_line = np.array([[1,-x_x*sin(phi)+x_y*sin(phi)],[0,1]])
                
        return np.array([rho_,phi]), H_tf, H_line
                
    #========================================================================
    def lineDist(self,z,idx):
        '''
        Given a line and an index of the state vector it computes the
        distance between both lines
        '''        
        # TODO program this function
        num_fea = self.get_number_of_features_in_map()

        # Transform the map line into robot frame and compute jacobians
        x_inv = compInv(self.xk[0:3])
        #print(x_inv)

        #print('xkshape', self.xk.shape)
        #print('idx', idx)
        p = self.tfPolarLine(x_inv[0], self.xk[idx:idx+2])
        h = p[0]
        H_position = p[1]
        H_line = p[2]
        
        # Allocate overall jacobian
        H = np.zeros((2,3+2*num_fea))
        
        # Concatenate position jacobians and place them into the position
        H_1 = np.dot(H_position, x_inv[1])
        H[:, 0:3] = H_1

        # Place the position of the jacobina with respec to the line in its
        # position
        H[:, idx:idx+2] = H_line

        # Calculate innovation
        v = z-h
        
        # Calculate innovation uncertainty
        S = np.dot(np.dot(H,self.Pk), np.transpose(H)) + self.Rk
  
        # Calculate mahalanobis distance
        D = np.dot(np.dot(np.transpose(v), inv(S)), v)
        
        return D,v,h,H
