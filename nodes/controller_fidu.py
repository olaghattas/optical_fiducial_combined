#!/usr/bin/env python

#import the dependencies
 
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import numpy as np
import tf2_ros
import tf.transformations 
import tf2_geometry_msgs


class Jackal:
	def __init__(self, K_gains):
		self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.sub_img_detec =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.detection_callback)

		self.vel = Twist()
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.saved_time = rospy.Time.now()
		self.point = None

    def apriltag_callback(self,msg):
        apriltags_list = list()
        
        if msg.detections:
            '''If there's an AprilTag in the image'''
            min_distance = np.inf
            selected_id = 0
            selected_apriltag = None
            for at in msg.detections:
            #     dist = np.linalg.norm([tmp_point.x, tmp_point.y, tmp_point.z])
            #     if dist < min_distance:
            #         min_distance = dist
            #         selected_id = at.id[0]
            #         selected_apriltag = at.pose.pose.pose

                #change frame from camera to baselink
                source_frame = "front_realsense_gazebo"
                transform = self.tfBuffer.lookup_transform("base_link", source_frame, rospy.Time(0), rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(selected_apriltag, transform)
                '''Now add the apriltag to seen apriltags_list'''
                apriltags_list.append([at.id[0],pose_transfromed.pose.pose.pose])

            self.apriltags_list = apriltags_list
            # robot coordinates in se(3)
            # at_to_robot = tu.msg_to_se3(selected_apriltag)
            
        else:
            rospy.logwarn("Can't find an AprilTag in the image!")
            self.apriltags_list = list()
    
    def dist_robot_landmark(self, rot_w_at): # rot_thisframe_inthisframe
        y = []
        for at in self.apriltags_list: # at is tag.pose.pose.pose
            at_position = at.position
            # distance from apriltag to robot 
            dist = np.linalg.norm([at_position.x,at_position.y, at_position.z])
            # rotation matrix of apriltag and robot 
            rot_rob_at = tu.msg_to_se3(at)[:3,:3]
            y = rot_w_at@(rot_rob_at).T
            y.append(y[:2,3]) # should be of size 2,1
        return y

    def compute_u(self,rot_w_at): # n = num of aptags seen K=[?,2n] y = [2n,1]
        y = self.dist_robot_landmark(rot_w_at)
        K = 0
        k = 1
        u = K@y.reshape(-1,1) + k
        return u

    def to_tf(self,pos,ori):
        return np.block([[np.array(ori),pos.reshape((-1,1))],[0,0,0,1]])

### To DO: get orientation

    def input_sparse(self):
        u = self.compute_u(rot_w_at)

        alpha = 0.5
        beta = 0.1

        ux = alpha/np.linalg.norm(u)
        ux = ux@[[np.cos(self.orien)],[np.sin(self.orien)]].T
        ux = ux@U

        wz = beta/np.linalg.norm(u)
        wz = wz@[[0],[0],[1]].T
        cross = np.cross([[np.cos(self.orien)],[np.sin(self.orien)],[0]],[[u],[0]])
        wz = wz@cross

        # adjust the velocity message
		self.vel.angular.z = wz
		self.vel.linear.x = ux
		#publish it
		self.pub.publish(self.vel)

## CHECK IF THIS ORIENTATION TO BE USED
    # def update_robot_pose(self,at_id,apriltag_to_robot):

    #     # Finally, robot pose wrt. the world: WTR = WTAT & (RTAT)-1
    #     self.pose = self.landmarks[at_id].tf.dot(np.linalg.inv(apriltag_to_robot))

    #     # Update the orientation vector of the robot from the seen AprilTag
    #     # because we only want the X vector
    #     orientation = self.pose[:3,0].flatten()
    #     orientation[2] = 0
    #     # Finally, update the orientation vector
    #     self.orientation = orientation
    #     self.orientation /= np.linalg.norm(self.orientation)

home_dir = os.environ["HOME"]

if __name__ == "__main__":

    rospy.init_node("jackal_iros_node")
    
    shared_path = os.environ["HOME"]+"/jackal_ws/src/optical_fiducial_combined/csv/"
    K_dir = shared_path + "K_gains.csv"
    K_gains = read_matrix(K_dir)

### TO DOO ARRAY WITH LANDMARK POSITION WRT WORLD
    jackal = Jackal(K_gains)

    r = rospy.Rate(10)
	while not rospy.is_shutdown():
        if len(jackal.apriltags_list):
		    jackal.input_sparse()
        else:
            print(no apriltags)
		r.sleep()

 


