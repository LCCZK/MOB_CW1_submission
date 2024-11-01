from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from . pf_base import PFLocaliserBase

from . util import rotateQuaternion, getHeading

import copy
import math
import random
import numpy as np


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self, logger, clock):
        # ----- Call the superclass constructor
        super().__init__(logger, clock)
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.2 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 2 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 2 # Odometry model y axis (side-to-side) noise
 
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 60   # Number of readings to predict

        # ----- Filter parameters
        # Basic params
        self.INIT_ESTIMATED_POSE_UNCERTANTY = (10, 10, math.pi)
        
        # Normal resampaling params
        self.POSE_STD_THRESHOLD_TO_RESAMPLE = (0.5, 0.5, 0.05*math.pi)
        self.RESAMPLE_NOISE = (0.3, 0.3, 0.1*math.pi)
        
        # Noisy resampaling params
        self.POSE_STD_THRESHOLD_TO_NOISY_RESAMPLE = (2, 2, 0.5*math.pi)
        self.MEAN_PARTICLE_WEIGHT_THRESHOLD_TO_NOISY_RESAMPLE = 10
        self.NOISY_RESAMPLE_NOISE = (10, 10, 0.5*math.pi)

        self.MIN_NEW_PARTICLE_WEIGHT = 4
        self.MAX_GENERATE_ATTEMPT = 5

        self.ESTIMATION_CLUSER_POSITION_RANGE = (2,2) #(x,y)

        

    #Logger functions
    def red_info_logging(self,msg):    
        red_info_color="\x1b[41m"
        default_info_color="\x1b[40m\x1b[37m"
        self._logger.info(red_info_color + msg + default_info_color)

    #Logger functions
    def yellow_info_logging(self,msg):    
        yellow_info_color="\x1b[43m\x1b[30m"
        default_info_color="\x1b[40m\x1b[37m"
        self._logger.info(yellow_info_color + msg + default_info_color)

    #Logger functions
    def blue_info_logging(self,msg):    
        blue_info_color="\x1b[46m\x1b[30m"
        default_info_color="\x1b[40m\x1b[37m"
        self._logger.info(blue_info_color + msg + default_info_color)

    def get_random_pose(self, mean:Pose, noise_range:tuple) -> Pose:
        random_particle = Pose()
        random_particle.position.x = ((random.gauss(mu=mean.position.x, sigma=0.2 * noise_range[0])))
        random_particle.position.y = ((random.gauss(mu=mean.position.y, sigma=0.2 * noise_range[1])))
        random_particle.orientation = rotateQuaternion(mean.orientation, (random.gauss(mu=0.0, sigma=0.2) * noise_range[2]))
        
        return random_particle
            
    def initialise_particle_cloud(self, initialpose:PoseWithCovarianceStamped) ->PoseArray:
        self.PC_converged = False
        self.odom_initialised = False
        pose = initialpose.pose.pose

        self.prev_odom_x = pose.position.x
        self.prev_odom_y = pose.position.y
        self.prev_odom_heading = getHeading(pose.orientation)

        self.particle_importance = np.ones(self.NUMBER_PREDICTED_READINGS)
        self.mean_particle_weight = 1

        init_particlecloud = PoseArray()
        for i in range(0,self.NUMBER_PREDICTED_READINGS):
            init_particlecloud.poses.append(
                self.get_random_pose(pose, self.INIT_ESTIMATED_POSE_UNCERTANTY))  
        
        return init_particlecloud

    def update_particle_cloud(self, scan:LaserScan):
        if scan.ranges != None:
            scan.ranges = self.valid_scan(scan)
            # self.red_info_logging(str(scan))      
            self.particle_importance = self.update_particle_importance(scan)
            if (self.PC_converged and ((self.get_PC_pose_std(self.particlecloud) > self.POSE_STD_THRESHOLD_TO_NOISY_RESAMPLE) or 
                                    self.mean_particle_weight < self.MEAN_PARTICLE_WEIGHT_THRESHOLD_TO_NOISY_RESAMPLE )):
                self.particlecloud, self.particle_importance = self.systematic_resampling(scan, self.NOISY_RESAMPLE_NOISE)
                self.PC_converged = False
                self.red_info_logging("Pose std threshold reached doing noisy resampling..")
                self.red_info_logging("current mean is %f"%self.mean_particle_weight)

            elif (self.get_PC_pose_std(self.particlecloud) > self.POSE_STD_THRESHOLD_TO_RESAMPLE):
                self.particlecloud, self.particle_importance = self.systematic_resampling(scan, self.RESAMPLE_NOISE)
                self.blue_info_logging("Doing normal resampaling")
                self.blue_info_logging("current mean is %f"%self.mean_particle_weight)

            else:
                self.yellow_info_logging("Pose std threshold not reached no resampling needed..")
                self.PC_converged = True
    
    def valid_scan(self, scan:LaserScan):
        clean_laser_range = []
        for s in scan.ranges:
            if math.isnan(s) :
                clean_laser_range.append(self.sensor_model.scan_range_max)
            else:
                clean_laser_range.append(s)
        return clean_laser_range


    def update_particle_importance(self, scan):
        indvidual_particle_weights = []

        for particle in self.particlecloud.poses:
            indvidual_particle_weights.append(self.sensor_model.get_weight(scan, particle))

        list_min=min(indvidual_particle_weights) - 1
        self.mean_particle_weight = np.mean(indvidual_particle_weights)

        return np.divide(np.add(self.particle_importance, np.subtract(indvidual_particle_weights, list_min)), 2) 


    def systematic_resampling(self,scan, resample_noise):
        self.yellow_info_logging("using systematic resampling")

        imp_mean = self.particle_importance.mean()
        imp_threshold = 0
        imp_counter = 0

        new_particle = Pose()
        new_particlecloud = PoseArray()
        new_particle_weight = 0
        regenerate_counter = 0

        new_particle_importance = []
        

        for particle, imp in zip(self.particlecloud.poses, self.particle_importance):
            imp_threshold += imp

            while (imp_threshold - imp_counter) >= imp_mean - 0.1:
                new_particle = self.get_random_pose(particle, resample_noise)
                new_particle_weight = self.sensor_model.get_weight(scan, new_particle)

                while new_particle_weight <= self.MIN_NEW_PARTICLE_WEIGHT:
                    new_particle = self.get_random_pose(particle, (4, 4, 0.5*math.pi))
                    new_particle_weight = self.sensor_model.get_weight(scan, new_particle)
                    regenerate_counter+=1

                    if regenerate_counter > self.MAX_GENERATE_ATTEMPT:
                        new_particle = self.get_random_pose(self.get_most_impotrant_partical(), (5, 5, 0.5*math.pi))
                        new_particle_weight = self.sensor_model.get_weight(scan, new_particle)
                
                new_particlecloud.poses.append(copy.deepcopy(new_particle))
                new_particle_importance.append(new_particle_weight)
                imp_counter += imp_mean
                regenerate_counter = 0

        new_particle_importance_min = min(new_particle_importance) + 1
        self.mean_particle_weight = np.mean(new_particle_importance)

        
        return copy.deepcopy(new_particlecloud), np.subtract(new_particle_importance, new_particle_importance_min)


    def estimate_pose(self) -> Pose:
        fintered_pose, filtered_imp = self.get_cluster_with_partical(self.get_most_impotrant_partical(), self.ESTIMATION_CLUSER_POSITION_RANGE)
        return self.weighted_estimated_pose(fintered_pose, filtered_imp)
    

    def get_PC_pose_std(self, particlecloud) -> tuple:
        poses = particlecloud.poses
        x = []
        y = []
        r = []

        for pose in poses:
            x.append(pose.position.x)
            y.append(pose.position.y)
            r.append(getHeading(pose.orientation))

        return (np.std(x), np.std(y), np.std(r))
    
    def get_cluster_with_partical(self, k_pose:Pose, cluster_radius:tuple):
        K_pose_list = (k_pose.position.x, k_pose.position.y)
        cluster = PoseArray()
        imp = []

        for pose, i in zip(self.particlecloud.poses, self.particle_importance):
            if ((abs(pose.position.x - K_pose_list[0]) < cluster_radius[0]) and 
                (abs(pose.position.y - K_pose_list[1]) < cluster_radius[1])):
                cluster.poses.append(pose)
                imp.append(i)
        
        return cluster, imp
    
    def get_most_impotrant_partical(self):
        return self.particlecloud.poses[self.particle_importance.argmax()]
    
    def weighted_estimated_pose(self, particlecloud_cluster:PoseArray, imp) -> Pose:
        estimatedpose = Pose()
        impotance_normaliser = sum(imp)
        poses = []

        for particle in particlecloud_cluster.poses:
            poses.append([particle.position.x, particle.position.y, 
                          particle.orientation.x, particle.orientation.y, particle.orientation.z, particle.orientation.w])

        weighted_pose = [0,0,0,0,0,0]

        for pose, i in zip(poses, imp):
            weighted_pose[0] += (pose[0] * i) / impotance_normaliser
            weighted_pose[1] += (pose[1] * i) / impotance_normaliser
            weighted_pose[2] += (pose[2] * i) / impotance_normaliser
            weighted_pose[3] += (pose[3] * i) / impotance_normaliser
            weighted_pose[4] += (pose[4] * i) / impotance_normaliser
            weighted_pose[5] += (pose[5] * i) / impotance_normaliser


        estimatedpose.position.x = weighted_pose[0]
        estimatedpose.position.y = weighted_pose[1]
        estimatedpose.orientation.x = weighted_pose[2]
        estimatedpose.orientation.y = weighted_pose[3]
        estimatedpose.orientation.z = weighted_pose[4]
        estimatedpose.orientation.w = weighted_pose[5]

        return estimatedpose

     

    

        

#NOT USING
    # def filter_outliners(self, particlecloud:PoseArray) -> PoseArray:
    #     filtered_particlecloud = PoseArray()
    #     poses = particlecloud.poses
    #     p = []
        
    #     for pose in poses:
    #         p.append([pose.position.x, pose.position.y, getHeading(pose.orientation)])

    #     p_mean = np.mean(p, axis=0)


    #     x_lq, x_uq = np.quantile(p[0],[0.25,0.75])
    #     y_lq, y_uq  = np.quantile(p[1],[0.25,0.75])
    #     r_lq, r_uq  = np.quantile(p[2],[0.25,0.75])

    #     x_iqr = x_uq - x_lq
    #     y_iqr = y_uq - y_lq
    #     r_iqr = r_uq - r_lq

    #     for pose in poses:
    #         if (pose.position.x > (p_mean[0] - x_iqr)and pose.position.x < (p_mean[0] + x_iqr) and
    #             pose.position.y > (p_mean[1] - y_iqr)and pose.position.y < (p_mean[1] + y_iqr) and
    #             getHeading(pose.orientation) > (p_mean[2] - r_iqr) and getHeading(pose.orientation) < (p_mean[2] - r_iqr)):
    #             filtered_particlecloud.poses.append(pose)

    #     return filtered_particlecloud


    # def Gaussian_dist(self, pose:list , mean:list , cov:np.matrix) -> float:
    #     self.yellow_info_logging("%s"%str(cov))
    #     pose_v=np.matrix(pose).T
    #     mean_v=np.matrix(mean).T
    #     # sensor_noise_cov = np.matrix([[self.ODOM_TRANSLATION_NOISE/2, 0], [0, self.ODOM_ROTATION_NOISE/2]])
    #     pose_m_mju = np.subtract(pose_v, mean_v)
    #     pose_m_mju_t = pose_m_mju.T
    #     cov_det = np.linalg.det(cov)
    #     cov_inv = np.linalg.inv(cov)

    #     return (math.exp((-1/2) * 
    #                     float(np.matmul(pose_m_mju_t, np.matmul(cov_inv, pose_m_mju)))
    #                     ) / math.sqrt(((2*math.pi)**2)*cov_det))

        # def normal_estimated_pose(self, particlecloud_cluster:PoseArray) -> Pose:
    #     estimatedpose = Pose()
    #     # impotance_normaliser = sum(particle_importance)
    #     poses = []

    #     for particle in particlecloud_cluster.poses:
    #         poses.append([particle.position.x, particle.position.y, 
    #                       particle.orientation.x, particle.orientation.y, particle.orientation.z, particle.orientation.w])
            

    #     poses_mean = np.mean(poses, axis=0)        


    #     estimatedpose.position.x = poses_mean[0]
    #     estimatedpose.position.y = poses_mean[1]
    #     estimatedpose.orientation.x = poses_mean[2]
    #     estimatedpose.orientation.y = poses_mean[3]
    #     estimatedpose.orientation.z = poses_mean[4]
    #     estimatedpose.orientation.w = poses_mean[5]

    #     return estimatedpose