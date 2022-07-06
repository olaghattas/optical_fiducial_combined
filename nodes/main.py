#!/usr/bin/env python

#import the dependencies
 
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import numpy as np
import transformation_utilities as tu
import so3_utilities as so
import transformations as tr
import tf2_ros
import tf.transformations 
import tf2_geometry_msgs
from vision_based_nav import vision_based_nav
from bearingRRTLPplanning import bearingRRTLPplanning


##TODO : 
## 1- detect the april tag


class Main():
    def __init__(self) -> None:
        self.vision_based_nav = vision_based_nav()
        self.bearingRRTLPplanning = bearingRRTLPplanning()
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.sub_img_detec =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.detection_callback)
		# Odometry will be (x,y,theta)
        self.odometry = None
        self.detected = None
        self.start_point = [0,0]
        self.apriltags_list = list()
        if not SIMULATION:
            self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odometry_callback)       
    
    
    # def odometry_callback(self,msg):
        
    #     if self.odometry is not None:

    #         self.odometry[0] = self.start_point[0,0] + msg.pose.pose.position.x
    #         self.odometry[1] = self.start_point[1,0] + msg.pose.pose.position.y

    #         q = msg.pose.pose.orientation
    #         orientation_matrix = tr.quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
    #         theta = np.arctan2(orientation_matrix[1,0],orientation_matrix[0,0])
    #         self.odometry[2] = self.odometry[2] + theta

    #     self.linear_speed = np.linalg.norm(np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]))
    #     self.angular_speed = np.linalg.norm(np.array([msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]))
    
    
    def detection_callback(self,msg):
        apriltags_list = list()

        if msg.detections:
            '''If there's an AprilTag in the image'''
            min_distance = np.inf
            selected_id = 0
            selected_apriltag = None

            for at in msg.detections:

                tmp_point = at.pose.pose.pose.position

                dist = np.linalg.norm([tmp_point.x, tmp_point.y, tmp_point.z])
                if dist < min_distance:
                    min_distance = dist
                    selected_id = at.id[0]
                    selected_apriltag = at.pose.pose.pose
                    selected_apriltag.position = tmp_point

                if at.id[0] not in self.K_to_landmark[self.current_K]:
                    continue
                
                '''Now add the apriltag to seen apriltags_list'''
                #print("\nApriltags we want to see: ", self.K_to_landmark[self.current_K])
                #print("\nApriltags that we see: ", at.id[0])
                apriltags_list.append([at.id[0],tu.msg_to_se3(at.pose.pose.pose)])
                #print("Adding {} to the seen apriltags".format(at.id[0]))

                if len(apriltags_list) == 2:
                    self.apriltags_list = apriltags_list
                    #print("Apriltags that we select: ", self.apriltags_list)
                    self.detected = True
                    self.on_apriltag_detection()
                    break


            if len(apriltags_list) < 2:
                print("I'm seeing these landmarks: ",apriltags_list)
                rospy.logwarn("I'm not seeing enough apriltags!")
                self.apriltags_list = list()
                self.detected = False
                self.on_apriltag_detection()

            ### Update robot pose
            # Use the closest apriltag to update my position
            at_rel_pose = tu.msg_to_se3(selected_apriltag)
            at_to_robot = self.apriltag_to_robot(at_rel_pose)

            self.update_robot_pose(selected_id,at_to_robot)

        else:
            rospy.logwarn("Can't find an AprilTag in the image!")
            self.apriltags_list = list()
            self.detected = False
            self.on_apriltag_detection()

    def apriltag_to_robot(self,at_rel_pose):
        # return self.cam_to_robot.dot(np.block([[self.apriltag_transform.dot(at_rel_pose[:3,:3]),at_rel_pose[:3,3][:,None]],[0,0,0,1]]))
        return self.cam_to_robot.dot(np.block([[at_rel_pose[:3,:3].dot(self.apriltag_transform),at_rel_pose[:3,3][:,None]],[0,0,0,1]]))

    def on_apriltag_detection(self):
        if self.detected:
            self.bearingRRTLPplanning.activate()
            self.vision_based_nav.detactivate()
        else:
            self.vision_based_nav.activate()
            self.bearingRRTLPplanning.deactivate()
        