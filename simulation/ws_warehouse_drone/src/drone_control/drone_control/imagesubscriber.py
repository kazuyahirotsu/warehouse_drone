# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import struct
import open3d as o3d

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    ## This is for visualization of the received point cloud.
    self.vis = o3d.visualization.Visualizer()
    self.vis.create_window()
    self.o3d_pcd = o3d.geometry.PointCloud()

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/camera', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription_depth = self.create_subscription(
      PointCloud2, 
      '/depth_camera/points', 
      self.listener_callback_depth, 
      10)
    self.subscription_depth # prevent unused variable warning

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription_depth_image = self.create_subscription(
      Image, 
      '/depth_camera', 
      self.listener_callback_depth_image, 
      10)
    self.subscription_depth_image # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    # image = current_frame

    image = cv2.resize(current_frame, (960, 540)) 

    cv2.imshow('Detected Frame', image)    
    cv2.waitKey(1)

  def listener_callback_depth(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving depth frame')

    pcd_as_numpy_array = np.array(list(read_points(data)))

    self.vis.remove_geometry(self.o3d_pcd)

    try:
        
        # Extract XYZ coordinates
        xyz = pcd_as_numpy_array[:, :3]  # Assuming the first three columns are XYZ

        # Create Open3D point cloud
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd.points = o3d.utility.Vector3dVector(xyz)  # Set XYZ data
        
        # If the fourth value is intensity and you want to visualize it as grayscale
        # You would typically normalize the intensity and then set it
        # pcd.colors = o3d.utility.Vector3dVector(np.tile(intensity, (1, 3)))  # Assuming the fourth column is intensity
        
        # If the fourth value is RGB packed into a single float, you need to unpack it
        # This requires knowing the exact packing format and may involve bit operations

        # Visualize the point cloud (optional, if you want to see it within the script)
        # o3d.visualization.draw_geometries([self.o3d_pcd])
        
    except Exception as e:
        print("Encountered an error:", e)

    # The rest here is for visualization.

    self.vis.add_geometry(self.o3d_pcd)
    self.vis.poll_events()
    self.vis.update_renderer()

  def listener_callback_depth_image(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving depth image frame')

    depth_image = self.br.imgmsg_to_cv2(data, desired_encoding='32FC1')
    depth_image_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_image_visual = np.uint8(depth_image_visual)
    cv2.imshow('Detected Depth Image Frame', depth_image_visual)
    cv2.waitKey(1)
    print(f"Depth range: min {np.min(depth_image)}, max {np.max(depth_image)}")


    # depth_image = self.br.imgmsg_to_cv2(data, desired_encoding='32FC1')

    # cv2.imshow('Detected Depth Image Frame', depth_image)    depth
    # cv2.waitKey(1)



## The code below is "ported" from 
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
# I'll make an official port and PR to this repo later: 
# https://github.com/ros2/common_interfaces
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset) #?????
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()