#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node

# Realsense Node:
class Realsense(Node):
    def __init__(self, fps= 60):
        super().__init__('realsense')

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        self.i=0

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        
        print( f"Connect: {device_product_line}" )
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
             found_rgb = True

        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)
            
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
        self.config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)
        
        is0k= True

        #Start Streaming
        self.pipeline.start(self.config)
        self.image_publisher = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth', 10)
        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)
        self.timer = self.create_publisher(0.001, self.activate) #0.1 seconds to target a frequency of 100 hertz
        self.bridge = CvBridge()

    def activate(self):
        frames = self.pipeline.wait_for_frames()
        
        self.i=self.i + 1
        
        depth_frame = frames.first(rs.stream.depth)
        color_frame = frames.first(rs.stream.color)
        infra_frame_1 = frames.get_infrared_frame(1)
        infra_frame_2 = frames.get_infrared_frame(2)
        
        ir = frames.first(rs.stream.infrared)
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        infra_image_1 = np.asanyarray(infra_frame_1.get_data())
        infra_image_2 = np.asanyarray(infra_frame_2.get_data())
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        infra_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_1, alpha=0.7), cv2.COLORMAP_JET)
        infra_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_2, alpha=0.7), cv2.COLORMAP_JET)
        
        msg_image = self.bridge.cv2_to_imgmsg(color_image,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)
        
        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "depth"
        self.depth_publisher.publish(msg_depth)
        
        msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_1,"bgr8")
        msg_infra.header.stamp = msg_image.header.stamp
        msg_infra.header.frame_id = "infrared_1"
        self.infra_publisher_1.publish(msg_infra)
        
        msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_2,"bgr8")
        msg_infra.header.stamp = msg_image.header.stamp
        msg_infra.header.frame_id = "infrared_2"
        self.infra_publisher_2.publish(msg_infra)
        
        
        
        
        
# Node processes:
def main(args=None):
    rclpy.init(args=args)
    rsNode = Realsense()
    rclpy.spin(rsNode)
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()