#!/usr/bin/env python

import rospy
import baxter_interface
from std_msgs.msg import String
from baxter_proj2.srv import *
from baxter_proj2.msg import *



def control(data):
        

def MoveRobot(Action, Target):
    # TODO move robot     
    rospy.wait_for_service('move_robot')
    service_move = rospy.ServiceProxy('move_robot', move_robot)
    try: 
        response = service_move(Action, Target)
        if response == 'False':
            rospy.loginfo("Wrong Operation")
        else:
            rospy.loginfo("Successful " + Action)
    except rospy.ServiceException, e:
            print "Service call failed: %s" %e 
    
def ros_controller():
    # initial node naming controller
    rospy.init_node('controller')

    # set the name of subsciber to ros_singal
    rospy.Subscriber("control_signal", String, control)

    rospy.wait_for_service('move_robot')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    ros_controller()
