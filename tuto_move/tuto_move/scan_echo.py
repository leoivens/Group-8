#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
import math
from geometry_msgs.msg import *

class ScanInterpret(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.points = []
        self.scan_publisher = self.create_publisher(PointCloud, '/scan_node', 10)

    def scan_callback(self, scanMsg):
        pointcloud = PointCloud()
        angle= scanMsg.angle_min
        for aDistance in scanMsg.ranges :
            if 0.1 < aDistance and aDistance < 5.0 :
                aPoint= Point32()
                aPoint.x= (float)(math.cos(angle) * aDistance)
                aPoint.y= (float)(math.sin(angle) * aDistance)
                aPoint.z= (float)(0)
                pointcloud.points.append(aPoint)
            angle+= scanMsg.angle_increment
        
        pointcloud.header.frame_id = "laser"
        self.scan_publisher.publish(pointcloud)

def main(args=None):
    rclpy.init(args=args)
    scanInterpret = ScanInterpret()
    rclpy.spin(scanInterpret)
    scanInterpret.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
