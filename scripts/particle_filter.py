#!/usr/bin/env python3

import math
from random import randint, random, choices

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from numpy.random import random_sample
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from tf import TransformBroadcaster, TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""
    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])
    return yaw

class Particle:
    def __init__(self, pose, w):
        # Particle pose (Pose object from geometry_msgs)
        self.pose = pose
        # Particle weight
        self.w = w

class ParticleFilter:
    def __init__(self):
        # Once everything is setup initialized will be set to true
        self.initialized = False

        # Initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # Set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"
        # Inialize our map
        self.map = OccupancyGrid()
        # The number of particles used in the particle filter
        self.num_particles = 1000
        # Initialize the particle cloud array
        self.particle_cloud = []
        # Initialize the estimated robot pose
        self.robot_estimate = Pose()
        # Set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.05
        self.ang_mvmt_threshold = (np.pi / 24)
        self.odom_pose_last_motion_update = None

        #### Custom parameters
        # Range of initial particle distribution
        self.x_range = [-.2, 3]
        self.y_range = [-3, .2]
        # Gaussian sigma for randomizing new particle poses
        self.x_gauss = 0.1
        self.y_gauss = 0.1
        self.theta_gauss = 0.2

        # Setup publishers and subscribers
        # Publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)
        # Publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)
        # Subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)
        # Subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)
        # Enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # Intialize the particle cloud
        self.initialize_particle_cloud()
        self.initialized = True

    def get_map(self, data):
        self.map = data

    def initialize_particle_cloud(self):
        self.particle_cloud = []

        for i in range(self.num_particles):
            # Randomly set position and heading within the maze
            x = np.random.uniform(*self.x_range)
            y = np.random.uniform(*self.y_range)
            theta = np.random.uniform(0, 2*np.pi)

            p = Pose()

            # Set the location parameters
            p.position = Point()
            p.position.x = x
            p.position.y = y
            p.position.z = 0

            # Set the rotation quaternion based on the heading angle
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, theta)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # initialize the new particle, where all will have the same weight (1.0)
            particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(particle)

        self.normalize_particles()
        self.publish_particle_cloud()

    def normalize_particles(self):
        # Find the total weight of all particles
        total = sum(map(lambda x:x.w, self.particle_cloud))

        # Normalize the weights, making all the particle weights sum to 1.0
        # Assume equal weight if the total was 0 (helped handle unexpected bugs)
        for particle in self.particle_cloud:
            particle.w = particle.w / total if total>0 else 1/len(self.particle_cloud)

    def publish_particle_cloud(self):
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)

    def publish_estimated_robot_pose(self):
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)

    def resample_particles(self):
        # Sample from the particle cloud
        particles = choices(self.particle_cloud, list(map(lambda x:x.w, self.particle_cloud)), k=self.num_particles)

        # Make a new particle cloud from the sampled particles
        self.particle_cloud = []
        for particle in particles:
            # Unfortunately, we created a bunch of pointers rather than new
            # particles, so we now have to create a bunch of new Particle()'s

            p = Pose()

            # Set the location parameters
            p.position = Point()
            # Add some Gaussian random noise to the new position
            p.position.x = particle.pose.position.x + np.random.normal(0, self.x_gauss, 1).item()
            p.position.y = particle.pose.position.y + np.random.normal(0, self.y_gauss, 1).item()
            p.position.z = 0

            # Set the rotation quaternion based on the heading angle
            p.orientation = Quaternion()
            theta = get_yaw_from_pose(particle.pose)
            # Add some Gaussian random noise to the new angle
            theta += np.random.normal(0, self.theta_gauss, 1).item()
            q = quaternion_from_euler(0.0, 0.0, theta)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

    def robot_scan_received(self, data):
        # Wait until initialization is complete
        if not(self.initialized):
            return

        # We need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # Wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # Calculate the pose of the laser distance sensor
        p = PoseStamped(header=Header(stamp=rospy.Time(0), frame_id=data.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # Determine where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=data.header.stamp, frame_id=self.base_frame), pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # We need to be able to compare the current odom pose to the prior odom pose
        # If there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:
            # Check to see if we've moved far enough to perform an update
            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            # Find the movement deltas
            x_moved = curr_x - old_x
            y_moved = curr_y - old_y
            yaw_moved = curr_yaw - old_yaw

            if (np.abs(x_moved) > self.lin_mvmt_threshold or
                np.abs(y_moved) > self.lin_mvmt_threshold or
                np.abs(yaw_moved) > self.ang_mvmt_threshold):

                # Calculate the distance and angle of motion relative to current heading
                distance = (x_moved**2 + y_moved**2)**0.5
                relative_direction = np.arctan2(y_moved, x_moved) - curr_yaw

                # This is where the main logic of the particle filter is carried out
                self.update_particles_with_motion_model(distance, relative_direction, yaw_moved)
                self.update_particle_weights_with_measurement_model(data)
                self.normalize_particles()
                self.resample_particles()
                self.update_estimated_robot_pose()
                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()
                self.odom_pose_last_motion_update = self.odom_pose

    def update_estimated_robot_pose(self):
        # Based on the particles within the particle cloud, update the robot pose estimate

        x = 0
        y = 0
        theta_x = 0
        theta_y = 0

        # Iterate over all particle positions and headings
        for particle in self.particle_cloud:
            x += particle.pose.position.x
            y += particle.pose.position.y

            theta = get_yaw_from_pose(particle.pose)
            theta_x += np.cos(theta)
            theta_y += np.sin(theta)

        # Finish finding the average position and heading
        x /= len(self.particle_cloud)
        y /= len(self.particle_cloud)
        theta_x /= len(self.particle_cloud)
        theta_y /= len(self.particle_cloud)
        theta = np.arctan2(theta_y, theta_x)

        # Set the estimated position from the averages
        self.robot_estimate.position.x = x
        self.robot_estimate.position.y = y
        q = quaternion_from_euler(0.0, 0.0, theta)
        self.robot_estimate.orientation.x = q[0]
        self.robot_estimate.orientation.y = q[1]
        self.robot_estimate.orientation.z = q[2]
        self.robot_estimate.orientation.w = q[3]

    def update_particle_weights_with_measurement_model(self, data):
        # Get the closest obstacles to the front, left, and right of the robot
        front = data.ranges[0]
        left  = data.ranges[90]
        right = data.ranges[270]

        # Are width and height in the right order?
        # They are the same value here, so hard to tell
        map = np.array(self.map.data).reshape((self.map.info.width, self.map.info.height))

        for particle in self.particle_cloud:
            # Find the heading of the particle
            theta = get_yaw_from_pose(particle.pose)
            run = math.cos(theta)
            rise = math.sin(theta)

            x = particle.pose.position.x
            y = particle.pose.position.y

            # Raycast from each particle to the walls in front and beside it
            particle_front = self.particle_distance(map, x, y, run, rise)
            particle_left =  self.particle_distance(map, x, y, -1 * rise, run)
            particle_right = self.particle_distance(map, x, y, rise, -1 * run)

            # Weight particles highly if they match the true lidar readings
            particle.w = 1 / (abs(front - particle_front) + abs(left - particle_left) + abs(right - particle_right))

    def interpolate(self, x, x1, x2, y1, y2):
        # Interpolates like so:
        #    x1    x    x2
        #    y1   out   y2
        return y1 + (x-x1)*(y2-y1)/(x2-x1)

    def particle_distance(self, map, x, y, run, rise):
        distance = 0

        # Perform a raycast from <x, y>, stepping
        # through the map in direction <run, rise>
        while True:
            # Make sure map index is valid
            if 0 > x > self.map.info.width:
                break
            if 0 > y > self.map.info.height:
                break

            # Get value in map
            v = map[
                int(self.interpolate(y, -3, 0, 140, 200)),
                int(self.interpolate(x,  0, 3, 200, 260))
            ]

            # Check map value
            if v == -1:
                distance = 100
            if v != 0:
                break

            # Break if something unexpected happens
            if distance > 100:
                break

            # Move forward a little bit
            x = x + (run*0.1)
            y = y + (rise*0.1)

            distance = distance + 0.1

        return distance

    def update_particles_with_motion_model(self, distance, relative_direction, yaw_moved):
        # Based on the how the robot has moved (calculated from its odometry), we'll move
        # all of the particles correspondingly
        for particle in self.particle_cloud:
            yaw = get_yaw_from_pose(particle.pose)

            # Move the particles the right distance at the relative heading
            theta = relative_direction + yaw
            particle.pose.position.x = particle.pose.position.x + distance * np.cos(theta)
            particle.pose.position.y = particle.pose.position.y + distance * np.sin(theta)

            # Update the change in the heading
            yaw = yaw + yaw_moved
            q = quaternion_from_euler(0.0, 0.0, yaw)
            particle.pose.orientation.x = q[0]
            particle.pose.orientation.y = q[1]
            particle.pose.orientation.z = q[2]
            particle.pose.orientation.w = q[3]

if __name__=="__main__":
    pf = ParticleFilter()
    rospy.spin()
