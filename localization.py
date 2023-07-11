from geometry_msgs.msg import Pose, PoseArray, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
import scipy as sp
from util import rotateQuaternion, getHeading
import random
import copy
from time import time

TAU = math.pi * 2


class PFLocaliser(PFLocaliserBase):
    # Initializing Parameters
    def __init__(self):
        # Call the Superclass Constructor
        super(PFLocaliser, self).__init__()

        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.01           # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.01      # Odometry model x-axis noise (forward)
        self.ODOM_DRIFT_NOISE = 0.01           # Odometry model y-axis noise (side-to-side)

   
        self.NUMBER_PREDICTED_READINGS = 30 

        self.total_number_particles = 750  

     
        self.initial_gauss_noise = 1 
        self.initial_vonmises_noise = 5  

       
        self.gauss_noise = 100  
        self.vonmised_noise = 50  


    def add_noise(self, pose):

        if self.gauss_noise > 5.0:
            self.gauss_noise -= 0.05
        else:
            self.gauss_noise = np.random.randint(0.05, 5) 
        
        pose.position.x += random.gauss(0, self.gauss_noise) * self.ODOM_TRANSLATION_NOISE 
        pose.position.y += random.gauss(0, self.gauss_noise) * self.ODOM_DRIFT_NOISE  
        pose.orientation = rotateQuaternion(pose.orientation, ((random.vonmisesvariate(0,self.vonmised_noise)) * self.ODOM_ROTATION_NOISE) )  # Add the noise to rotation angles
        return pose

    def initialise_particle_cloud(self, initialpose):
        cloud_array = PoseArray()
        for i in range(self.total_number_particles):
            initial_pose = Pose()
            initial_pose.position.x = initialpose.pose.pose.position.x + random.gauss(0,self.initial_gauss_noise) * self.ODOM_TRANSLATION_NOISE
            initial_pose.position.y = initialpose.pose.pose.position.y + random.gauss(0,self.initial_gauss_noise) * self.ODOM_DRIFT_NOISE
            initial_pose.orientation = rotateQuaternion(initialpose.pose.pose.orientation, ((random.vonmisesvariate(0, self.initial_vonmises_noise)) * self.ODOM_ROTATION_NOISE))
            cloud_array.poses.append(initial_pose)
        return cloud_array

    def update_particle_cloud(self, scan):
        
        global k
        cloud_array = PoseArray()
        probabilty = []
        weight = 0  
        for i in self.particlecloud.poses:  
            initial_weight = self.sensor_model.get_weight(scan, i)
            probabilty.append(initial_weight)
            weight += initial_weight

        for i in range(len(self.particlecloud.poses)):
            sum_values = random.random() * weight
            total_probability = 0
            k = 0
            while total_probability < sum_values:
                total_probability += probabilty[k]
                k = k + 1
            cloud_array.poses.append(copy.deepcopy(self.particlecloud.poses[k-1]))


        for i in (cloud_array.poses):
            i = self.add_noise(i)

        self.particlecloud = cloud_array

 
    def estimate_pose(self):

        estimated_pose = Pose()
        x_sum = 0
        y_sum = 0
        z_sum = 0
        w_sum = 0  

        for i in self.particlecloud.poses:
            x_sum += i.position.x 
            y_sum += i.position.y 
            z_sum += i.orientation.z 
            w_sum += i.orientation.w 

        estimated_pose.position.x = x_sum / self.total_number_particles
        estimated_pose.position.y = y_sum / self.total_number_particles
        estimated_pose.orientation.z = z_sum / self.total_number_particles
        estimated_pose.orientation.w = w_sum / self.total_number_particles

        orientations = [estimated_pose.position.x, estimated_pose.position.y, estimated_pose.orientation.z, estimated_pose.orientation.w]
        (r,p,y) = euler_from_quaternion(orientations)
        
        print('position in X coordinates: ', estimated_pose.position.x)
        print('position in y coordinates: ', estimated_pose.position.y)
        print('heading in degrees: : ', math.degrees(y))
        
        return estimated_pose
