import os
import sys
#import math
#import rospy
#import message_filters
#from std_msgs.msg import String
#from geometry_msgs import Pose

'''
class test_pose():

    def __init__(self):

        self._pose_pub = rospy.Publisher('robot_pose', Pose, queue_size=1)


    def callback(self):
        p = Pose()
        
        x, y, z = input("Enter a point (x,y,z): ").split()
        
        # get the position
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        
        # Make sure the quaternion is valid and normalized
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0 
        
        print("Point (x, y, z): {}, {}, {}".format(x, y, z))
        
        self._pose_pub.publish(p)
        
        
    def main(self):
        rospy.spin()


        
if __name__ == '__main__':

    rospy.init_node('test_pose_pub')
    tp = test_pose()
    tp.main()        
    
'''

while(1):
    x, y, z = input("Enter a point (x,y,z): ").split()
    
    x = float(x)
    y = float(y)
    z = float(z)
    
    print("Point (x, y, z): {}, {}, {}".format(x, y, z))
    
    