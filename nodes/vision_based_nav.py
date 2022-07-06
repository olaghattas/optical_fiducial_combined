#!/usr/bin/env python
import rospy
from controller import Controller
from optical_flow import OFCalculator
from tau_computation import TauComputationClass


class navigation():
    def __init__(self):
        pass

    def optical_flow(self):
        # if len(sys.argv) < 2:
        #     parameter = str(0)
        #     # print("Parameter = 1, verbose mode")
        # else:
        #     parameter = sys.argv[1]
        rospy.init_node("optical_flow", anonymous=False)
        OFCalculator("1")
        # rospy.spin()

    def tau_computation(self):
        rospy.init_node("tau_computation", anonymous=False)
        TauComputationClass()
        # rospy.spin()


    def controller(self):
        rospy.init_node("controller", anonymous=False)
        Controller()
        # rospy.spin()

    def activate(self):
        self.optical_flow()
        self.tau_computation()
        self.controller()

    def deactivate(self):
        pass

if __name__=="__main__":
	# #initialise the node
	rospy.init_node("ola", anonymous=True)
	navigation = navigation()
	#while the node is still on
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		navigation.activate()
		r.sleep()
