#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray

class welding_scene_publisher(Node):
    def __init__(self):
        super().__init__('welding_scene_publisher')
        
        self.marker_pub_ = self.create_publisher(MarkerArray, '/welding_scene_publisher/welding_scene', 10)
        self.timer_ = self.create_timer(1.0, self.publish_stls)

    def publish_stls(self):
        marker_array = MarkerArray()
        
        marker1 = self.create_stl_marker(
            id=0,
            mesh_path='file:///home/thibault/Documents/Cylindre.stl',
            pos_x=0.0, pos_y=-0.3, pos_z=0.0,
            ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0,
            scale_x=0.001, scale_y=0.001, scale_z=0.001,
            color_r=0.7, color_g=0.7, color_b=0.7, color_a=1.0
        )
        
        marker2 = self.create_stl_marker(
            id=1,
            mesh_path='file:///home/thibault/Documents/SoudureSelleCheval.stl',
            pos_x=0.0, pos_y=0.3, pos_z=0.0,
            ori_x=0.0, ori_y=0.0, ori_z=0.7071068, ori_w=0.7071068,
            scale_x=0.001, scale_y=0.001, scale_z=0.001,
            color_r=0.7, color_g=0.7, color_b=0.7, color_a=1.0
        )
        
        marker_array.markers.append(marker1)
        marker_array.markers.append(marker2)
        self.marker_pub_.publish(marker_array)

    def create_stl_marker(self, id, mesh_path, pos_x, pos_y, pos_z,
                         ori_x, ori_y, ori_z, ori_w,
                         scale_x, scale_y, scale_z,
                         color_r, color_g, color_b, color_a):
        
        marker = Marker()
        
        # Header
        marker.header.frame_id = "/nb/base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Marker properties
        marker.ns = "stl_objects"
        marker.id = id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = pos_x
        marker.pose.position.y = pos_y
        marker.pose.position.z = pos_z
        
        # Orientation
        marker.pose.orientation.x = ori_x
        marker.pose.orientation.y = ori_y
        marker.pose.orientation.z = ori_z
        marker.pose.orientation.w = ori_w
        
        # Scale
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        
        # Color
        marker.color.r = color_r
        marker.color.g = color_g
        marker.color.b = color_b
        marker.color.a = color_a
        
        # STL file path
        marker.mesh_resource = mesh_path
        marker.lifetime = Duration(seconds=0).to_msg()
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = welding_scene_publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
