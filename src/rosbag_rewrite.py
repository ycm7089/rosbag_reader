import rosbag
import rospy

from sensor_msgs.msg import PointCloud2, Imu, LaserScan
from nav_msgs.msg import Odometry


print("A")

read_bag = rosbag.Bag('/home/cm/rosbag_data/0315ranger_2.bag', 'r')
write_bag = rosbag.Bag('/home/cm/rosbag_data/new_0315ranger_2.bag', 'w')

print("B")

for topic, msg, time in read_bag.read_messages(topics=['/ouster/points','/ouster/imu', '/ranger_base_node/odom', '/maked_ouster_scan']) :
    
    if topic == '/ouster/points' :
        pc2 = PointCloud2()
        pc2 = msg

        write_bag.write('/ranger1/ouster/points', pc2, time)
    
    if topic == '/ouster/imu' :
        oimu = Imu()
        oimu = msg
        
        write_bag.write('/ranger1/ouster/imu', oimu, time)
    
    if topic == '/ranger_base_node/odom' :
        odom = Odometry()
        odom = msg

        write_bag.write('/ranger1/ranger_base_node/odom', odom, time)
    
    if topic == '/maked_ouster_scan' :
        lscan = LaserScan()
        lscan = msg
        
        write_bag.write('/ranger1/maked_ouster_scan', lscan, time)

read_bag.close()
write_bag.close()