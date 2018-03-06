#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from math import exp
from probabilistic_lib.functions import angle_wrap, get_polar_line
from random import random
import math

#===============================================================================
class ParticleFilter(object):
    '''
    Class to hold the whole particle filter.
    
    p_wei: weights of particles in array of shape (N,)
    p_ang: angle in radians of each particle with respect of world axis, shape (N,)
    p_xy : position in the world frame of the particles, shape (2,N)
    '''
    
    #===========================================================================
    def __init__(self, room_map, num, odom_lin_sigma, odom_ang_sigma, 
                 meas_rng_noise, meas_ang_noise,x_init,y_init,theta_init):
        '''
        Initializes the particle filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.map = room_map
        self.num = num
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        
        # Map
        map_xmin = np.min(self.map[:, 0])
        map_xmax = np.max(self.map[:, 0])
        map_ymin = np.min(self.map[:, 1])
        map_ymax = np.max(self.map[:, 1])
        
        # Particle initialization arround starting point
        self.p_wei = 1.0 / num * np.ones(num)
        self.p_ang =2 * np.pi * np.random.rand(num)
        self.p_xy  = np.vstack(( x_init+ 1*np.random.rand(num)-0.5,
                                 y_init+ 1*np.random.rand(num)-0.5 ))
        #Flags for resampling                         
        self.moving=False
        self.n_eff=0 #Initialize Efficent number as 0
    
    #===========================================================================
    def predict(self, odom):
        '''
        Moves particles with the given odometry.
        odom: incremental odometry [delta_x delta_y delta_yaw] in the vehicle frame
        '''
        #Check if we have moved from previous reading.
        if odom[0]==0 and odom[1]==0 and odom[2]==0:
            self.moving=False
        else:
            # TODO: code here!!
            # Add Gaussian noise to odometry measures
            delta_x_noisy = odom[0]+self.odom_lin_sigma*np.random.randn(self.num)
            delta_y_noisy = odom[1]+self.odom_lin_sigma*np.random.randn(self.num)
            delta_yaw_noisy = odom[2]+self.odom_ang_sigma*np.random.randn(self.num)

            # Transform from vehicle frame to world frame
            delta_x_noisy_w = delta_x_noisy * np.cos(-self.p_ang) + delta_y_noisy * np.sin(-self.p_ang)
            delta_y_noisy_w = -delta_x_noisy * np.sin(-self.p_ang) + delta_y_noisy * np.cos(-self.p_ang)

            # Increment particle positions in correct frame
            self.p_xy += np.vstack((delta_x_noisy_w, delta_y_noisy_w))

            # Increment angle
            self.p_ang += delta_yaw_noisy
            self.p_ang = angle_wrap(self.p_ang)
            
            #Update flag for resampling
            self.moving=True
                
    
    #===========================================================================
    def weight(self, lines):
        '''
        Look for the lines seen from the robot and compare them to the given map.
        Lines expressed as [x1 y1 x2 y2].
        '''

        # check point
        print('------------------------weighting------------------------------- ')
        # TODO: code here!!
        # Constant values for all weightings
        val_rng = 1.0 / (self.meas_rng_noise * np.sqrt(2 * np.pi))
        val_ang = 1.0 / (self.meas_ang_noise * np.sqrt(2 * np.pi))
        
        # Loop over particles
        for i in range(self.num):
            particle_pos = np.array([self.p_xy[0,i],self.p_xy[1,i],self.p_ang[i]])

            # Transform map lines to local frame and to [range theta]
            map_polar = np.zeros((self.map.shape[0],2))
            for j in range(self.map.shape[0]):
                map_polar[j,:] = get_polar_line(self.map[j,:], particle_pos)
            # Transform measured lines to [range theta] and weight them
            #print(map_polar)

            line_max_wei = np.zeros(lines.shape[0])
            for j in range(lines.shape[0]):
                #
                lines_polar = get_polar_line(lines[j,:])
                #print(lines_polar)
                #
                # Weight them
                for m in range(self.map.shape[0]):
                    wei_rng = val_rng*exp(-(lines_polar[0]-map_polar[m,0])**2/(2*(self.meas_rng_noise**2)))

                    wei_ang = val_ang*exp(-(lines_polar[1]-map_polar[m,1])**2/(2*(self.meas_ang_noise**2)))

                    # OPTIONAL question
                    # make sure segments correspond, if not put weight to zero
                    map_seg = math.sqrt((self.map[m,2]-self.map[m,0])**2 + (self.map[m,3]-self.map[m,1])**2)
                    line_seg = math.sqrt((lines[j,2]-lines[j,0])**2 + (lines[j,3]-lines[j,1])**2)

                    if line_seg > map_seg:
                        wei_rng = 0
                        wei_ang = 0

                    line_wei = wei_rng * wei_ang

                    if line_wei > line_max_wei[j]:
                        line_max_wei[j] = line_wei

            # Take best weighting (best associated lines)
            self.p_wei[i] *= np.prod(line_max_wei)
            

            
        # Normalize weights
        self.p_wei /= np.sum(self.p_wei)
        #print(self.p_wei)
        # TODO: Compute efficient number
        self.n_eff = 1.0/np.sum(self.p_wei**2)
        #print(self.n_eff)
        
    #===========================================================================
    def resample(self):
        '''
        Systematic resampling of the particles.
        '''
        # check point
        print('---------------------resampling--------------------------')
        # TODO: code here!!
        # Look for particles to replicate 
        r = random()*1.0/self.num
        c = self.p_wei[0]
        i = 0

        p_xy_new = np.zeros((self.p_xy.shape[0], self.p_xy.shape[1]))
        p_ang_new = np.zeros(self.p_ang.shape)
        for m in range(self.num):
            u = r + 1.0*m/self.num
            while u > c:
                i += 1
                c += self.p_wei[i]
            # Pick chosen particles
            p_xy_new[:,m] = self.p_xy[:,i]
            p_ang_new[m] = self.p_ang[i]
        
        self.p_xy = p_xy_new
        self.p_ang = p_ang_new
        self.p_wei = 1.0 / self.num * np.ones(self.num)
    
    #===========================================================================
    def get_mean_particle(self):
        '''
        Gets mean particle.
        '''
        # Weighted mean
        weig = np.vstack((self.p_wei, self.p_wei))
        mean = np.sum(self.p_xy * weig, axis=1) / np.sum(self.p_wei)
        
        ang = np.arctan2( np.sum(self.p_wei * np.sin(self.p_ang)) / np.sum(self.p_wei),
                          np.sum(self.p_wei * np.cos(self.p_ang)) / np.sum(self.p_wei) )
                          
        return np.array([mean[0], mean[1], ang])
