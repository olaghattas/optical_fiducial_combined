#!/usr/bin/env python

from __future__ import print_function

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import transformation_utilities as tu
import so3_utilities as so
import transformations as tr
import termcolor

import copy
import os

# # Check if I'm on a robot or a simulation computer:
# SIMULATION = False
# try:
#     import matplotlib.pyplot as plt
#     SIMULATION = True
# except:
#     print("I'll assume I'm in the real drone, so not simulating.")

# SIMULATION = False
    

def read_matrix(csv_dir):
    with open(csv_dir,'r') as f:
        return np.genfromtxt(f,delimiter=',')

def write_matrix(csv_dir,mat):
    with open(csv_dir,'w') as f:
        np.savetxt(f,mat,delimiter=',')

class Obstacle(object):
    def __init__(self,coordinates,transformation=np.eye(4)):
        self.coordinates = (transformation[:3,:3].dot(coordinates.T) + transformation[:3,3][:,None]).T

class Landmark(object):
    def __init__(self,id,position,orientation_type=0,transformation=np.eye(4),displaced=np.zeros((3,))):
        
        self.id = id
        self.position = transformation[:3,:3].dot(position) + transformation[:3,3] 

            
        self.orientation_type = orientation_type
        self.orientation = transformation[:3,:3].dot(ROTATION_TYPES[orientation_type])

        self.tf = self.to_tf(self.position,self.orientation)
        
        # We need to store the transformation between the seen landmark and the
        # algorithm landmark:
        self.visual_to_landmark(displaced)

    def to_tf(self,pos,ori):
        return np.block([[np.array(ori),pos.reshape((-1,1))],[0,0,0,1]])

    def dot(self,matrix):
        assert type(matrix) == np.ndarray
        return self.tf.dot(matrix)

    def visual_to_landmark(self,displaced):
        # Displacement is the position of the _seen_ landmark and not the _algorithm_ landmark:
        displacement = np.block([[self.orientation,displaced.reshape((3,-1))],[0,0,0,1]])
        self.displacement = np.linalg.inv(displacement).dot(self.tf)


class Jackal(object):

    def __init__(self,K_gains,landmarks,transform_matrix=np.eye(4),start_point = None,orientation_=[1,0,0],sequence_idx=0):

        self.transform_matrix = transform_matrix
        # One matrix with all gains concatenated
        self.K_gains = K_gains

        # Save landmark transformations wrt. mocap:
        self.landmarks = landmarks

        # Position and speed variables:
        self.start_point = copy.deepcopy(start_point)
        self.position = copy.deepcopy(start_point)
        self.orientation = np.array(orientation_)

        self.pose = np.block([[np.eye(3),self.position.reshape((-1,1))],[0,0,0,1]])
        

        self.linear_speed = np.inf
        self.angular_speed = np.inf
        self.speed = np.inf

        # robot to camera transform for the apriltags
        # self.cam_to_robot = (np.array([[0,0,1,-20],[-1,0,0,20.],[0,-1,0,-1],[0,0,0,1]],dtype=float))
        # self.cam_to_robot[:3,3] *= 1e-2

        self.K_to_landmark = read_matrix(home_dir + '/jackal_ws/src/jackal_iros_2022/csv/K_to_landmark.csv')
        # self.K_to_landmark = self.K_to_landmark[~np.isnan(self.K_to_landmark)]


        # For each trajectory, what K's will it visit?
        all_sequences = [[4,2],[19,18,16,14],[17,16,14],[7,6,5]]

        sequence_idx = 0
        
        self.sequence_id = sequence_idx
        self.K_sequence = all_sequences[sequence_idx]
        self.K_index = 0

        # self.current_cell = self.K_to_cell[self.K_sequence[self.K_index]]
        self.current_K = self.K_sequence[self.K_index]-2
        
        self.rate = rospy.Rate(5)

        self.apriltags_list = list()
        
        # # Odometry will be (x,y,theta)
        # self.odometry = None

        # subscribers and publishers
        self.apriltag_sub = rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.apriltag_callback,queue_size=1)
        
        # self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odometry_callback)

        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)   

    def apriltag_callback(self,msg):
        apriltags_list = list()
        
        if msg.detections:
            '''If there's an AprilTag in the image'''
            min_distance = np.inf
            selected_id = 0
            selected_apriltag = None
            for at in msg.detections:
                dist = np.linalg.norm([tmp_point.x, tmp_point.y, tmp_point.z])
                if dist < min_distance:
                    min_distance = dist
                    selected_id = at.id[0]
                    selected_apriltag = at.pose.pose.pose
                    #selected_apriltag.position = tmp_point

                #change frame from camera to baselink
                source_frame = "front_realsense_gazebo"
                transform = self.tfBuffer.lookup_transform("base_link", source_frame, rospy.Time(0), rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(selected_apriltag, transform)

                tmp_point = pose_transformed.pose.pose.pose.position

                # if the K's seen do not belong to the cell
                # if at.id[0] not in self.K_to_landmark[self.current_K]:
                #     continue
                
                '''Now add the apriltag to seen apriltags_list'''
                apriltags_list.append([at.id[0],tu.msg_to_se3(at.pose.pose.pose)])

            ### Update robot pose
            # Use the closest apriltag to update my position
            at_to_robot = tu.msg_to_se3(selected_apriltag)

            self.update_robot_pose(selected_id,at_to_robot)
            
        else:
            rospy.logwarn("Can't find an AprilTag in the image!")
            self.apriltags_list = list()
            
    # def apriltag_to_robot(self,at_rel_pose):
    #     # return self.cam_to_robot.dot(np.block([[self.apriltag_transform.dot(at_rel_pose[:3,:3]),at_rel_pose[:3,3][:,None]],[0,0,0,1]]))
    #     return self.cam_to_robot.dot(np.block([[at_rel_pose[:3,:3].dot(self.apriltag_transform),at_rel_pose[:3,3][:,None]],[0,0,0,1]]))


    def update_robot_pose(self,at_id,apriltag_to_robot):

        # Finally, robot pose wrt. the world: WTR = WTAT & (RTAT)-1
        self.pose = self.landmarks[at_id].tf.dot(np.linalg.inv(apriltag_to_robot))

        # Update the orientation vector of the robot from the seen AprilTag
        # because we only want the X vector
        orientation = self.pose[:3,0].flatten()
        orientation[2] = 0
        # Finally, update the orientation vector
        self.orientation = orientation
        self.orientation /= np.linalg.norm(self.orientation)
       
        
    def compute_displacements(self):
        robot_rotation = self.pose[:3,:3]

        y = list()

        for seen_at in self.apriltags_list:
            at_rel_pose = self.cam_to_robot.dot(seen_at[1])
            y_temp = robot_rotation.dot(at_rel_pose[:3,3])
            # y.append(y_temp/np.linalg.norm(y_temp))
            y.append(y_temp)

        return np.array(y).T

        
    def compute_control_input(self):

        if not self.apriltags_list:
            print("No apriltags!")
            return np.zeros((3,))

        landmarks_list_ids = self.K_to_landmark[self.current_K]
        landmarks_list_ids = landmarks_list_ids[~np.isnan(landmarks_list_ids)].flatten().tolist()

        # print("Expecting these apriltags:",landmarks_list_ids)

        if self.pose is None:
            return np.zeros((3,))
        
        if self.apriltags_list:
            '''self.apriltags_list should always be a list of 2 Apriltags'''
        
            seen_ids = (self.apriltags_list[0][0], self.apriltags_list[1][0])

            # Compute y_visible, i.e displacement measurements normalized --> AND in Mahroo's coordinates!
            # y_visible = GLOBAL_ROTATION_MATRIX.dot(self.compute_displacements())[:2,:]
            y_visible = self.compute_displacements()[:2,:]


        
        else:
            rospy.logwarn("I'm not seeing any useful landmark... returning 0.")
            #print("This is the old input I'm using: ", self.old_u)
            return np.zeros((3,))

        
        # We need the -2, as real K indices start with one and we didn't copy the null K [1]!
        k_index = self.K_sequence[self.K_index]-2
        
        # Obtain all landmarks in the cell for error computation:
        K_matrix = copy.deepcopy(self.K_gains[k_index*2:k_index*2+2,:])
        K_matrix = K_matrix[~np.isnan(K_matrix)].reshape((2,-1))

        #print("K_matrix: ", K_matrix)
        #print("landmarks: ", L)

        #print("Type of y_m (i.e. L): ", y_m.shape)

        u = K_matrix.dot(y_m.T.reshape(-1,1))
            
        # Finally, go back to global coordinates:
        # u = GLOBAL_ROTATION_MATRIX[:2,:2].T.dot(u.flatten())
        u = np.concatenate((u.flatten(),[0]))

        self.apriltags_list = list()

        return u


    def parse_inputs(self,u,orientation):

        # u = np.zeros((3,))

        linear_coef = 0.4/2
        angular_coef = 5.0/7 # fast, but slow enough so that we see landmarks
        
        linear_vel_threshold = 0.02*2/5
        
        linear_vel = linear_coef * u.dot(orientation)
        angular_vel = 0

        else:
            # In any normal case:
            if linear_vel < linear_vel_threshold:
                angular_vel = 1 * angular_coef * np.cross(orientation,u)[2]
            else:
                angular_vel = 1 * angular_coef * np.cross(orientation,u/np.linalg.norm(u))[2]
            linear_vel = linear_vel/np.linalg.norm(u)

        return linear_vel,angular_vel
    
    def navigate(self):

        # while (not rospy.is_shutdown() and self.position is None) and not SIMULATION:
        #     rospy.loginfo("Waiting for mocap to send robot position...")
        #     self.rate.sleep()
            
        self.old_u = np.zeros(3,)

        while not rospy.is_shutdown():
            # raise NotImplementedError("\nImplement:\nWaaay smaller speeds\nOdometry when not seeing apriltag\nAnything else?")
            u = self.compute_control_input()
          
            self.old_u = copy.deepcopy(u)
                
            # OR send /cmd_vel instead!
            vel_msg = Twist()
            
            # Obtain the linear and angular control inputs. self.parse_inputs contains all the parameter tuning required for the experiment:
            velocity_output = self.parse_inputs(u,self.orientation)
            if velocity_output is None:
                rospy.loginfo("Finished experiment!") 
                return
            elif velocity_output == 0:
                continue

            # If we've properly computed a linear and angular velocity:
            linear_vel,angular_vel = velocity_output 
     
            vel_msg.linear.x = linear_vel
            vel_msg.angular.z = angular_vel
                #print("Control input:\n",vel_msg.linear)
                
                # vel_msg = Twist()
                # print("Sending this velocity: ",vel_msg.linear)

                # print("Publishing this message:",vel_msg)

            self.vel_pub.publish(vel_msg)

            # Update simulated speed:
            self.linear_speed = linear_vel
            self.angular_speed = angular_vel
                                    
            self.rate.sleep()

    def on_rospy_shutdown(self):
        self.vel_pub.publish(Twist())
        rospy.logwarn("Stopping Jackal.")
        rospy.Rate(1).sleep()
        



home_dir = os.environ["HOME"]


if __name__ == "__main__":

    rospy.init_node("jackal_iros_node")

    shared_path = os.environ["HOME"]+"/catkin_ws/src/apriltag_gazebo/csv/"
    K_dir = shared_path + "K_gains.csv"
    landmark_positions_dir = shared_path + "landmark_positions.csv"
    landmark_orientations_dir = shared_path + "landmark_orientations_deformed.csv"
    transform_matrix_dir = shared_path + "transform_matrix.csv"

    visual_landmarks_dir = shared_path + "landmark_visual_positions.csv"

    try:
        visual_landmarks = read_matrix(visual_landmarks_dir)
    except Exception as e:
        raise e
        raise NotImplementedError("We don't have coordinates for visual landmarks yet!")

    K_gains = read_matrix(K_dir)
    landmark_positions = read_matrix(landmark_positions_dir)
    landmark_orientations = read_matrix(landmark_orientations_dir)
    transform_matrix = read_matrix(transform_matrix_dir) # From local to global

    landmarks_dict = make_my_landmarks(landmark_positions)

    # Save the coordinates in Mahroo's reference frames
    # write_matrix(shared_path + 'obstacle_coordinates_transformed.csv',obstacle_coordinates)
    # write_matrix(shared_path + 'landmark_coordinates_transformed.csv',landmark_positions)


    if SIMULATION:
        start_point = np.array([1.,4.,0.]).T
    else:
        start_point = None

    jackal = Jackal(K_gains,landmarks_dict,start_point=start_point,sequence_idx=2,orientation_=[0,-1,0])

    rospy.on_shutdown(jackal.on_rospy_shutdown)

    rospy.loginfo("jackal Jackald.")

    jackal.navigate()

    if SIMULATION:
        rospy.spin()