
#!/usr/bin/env python3

import rospy
import math
import tf
from tf.transformations import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class TF_pub:
    def __init__(self):
        
        # publihser
        self.odom_pose_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)
        
        self.odom_pose = Odometry()
        self.orientation_q = Odometry()
        self.orientation_list = []
        self.yaw = 0.0

        self.br = tf.TransformBroadcaster()


    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose.position
        
        self.orientation_q = msg.pose.pose.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (self.orientation_list)
        # print(self.yaw)
        self.tf_pub(self.yaw)
        

    def tf_pub(self, yaw):
        self.br.sendTransform((self.odom_pose.x, self.odom_pose.y, 0), tf.transformations.quaternion_from_euler(0, 0.0, self.yaw), rospy.Time.now(), "base_link", "map")
        # self.br.sendTransform((0.0, 0.0, 0), tf.transformations.quaternion_from_euler(0, 0.0, 0), rospy.Time.now(), "base_link", "odom")
        self.br.sendTransform((0.0, 0.0, 0.82), tf.transformations.quaternion_from_euler(0, 0.0, 0), rospy.Time.now(), "velodyne", "base_link")

    # br.sendTransform((0.0, 0.0, 0.72), tf.transformations.quaternion_from_euler(0.0, 0), rospy.Time.now(), "safe_link", "marker")


def main():
    rospy.init_node('tf_publisher')
    while not rospy.is_shutdown():        
        cm_tf = TF_pub()
    
if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        print("ERROR")