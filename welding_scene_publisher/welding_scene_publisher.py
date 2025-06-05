#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray

CYLINDRE_PATH = 'file://' + os.path.join('..', 'scene_meshes', 'Cylindre.stl')
SADDLE_PATH = 'file://' + os.path.join('..', 'scene_meshes', 'SoudureSelleCheval.stl')

class welding_mesh_publisher(Node):
    def __init__(self):
        super().__init__('welding_mesh_publisher')
        
        self.marker_pub_ = self.create_publisher(MarkerArray, '/welding_mesh_publisher/welding_mesh', 10)
        self.timer_ = self.create_timer(1.0, self.publish_stls)

    def publish_stls(self):
        marker_array = MarkerArray()
        
        cylindre = self.create_stl_marker(
            id=0,
            mesh_path=CYLINDRE_PATH,
            pos_x=0.0, pos_y=0.0, pos_z=0.0,
            ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0,
            scale_x=1.0, scale_y=1.0, scale_z=1.0,
            color_r=0.7, color_g=0.7, color_b=0.7, color_a=1.0
        )
        
        saddle = self.create_stl_marker(
            id=1,
            mesh_path=SADDLE_PATH,
            pos_x=2.0, pos_y=0.0, pos_z=0.0,
            ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0,
            scale_x=1.0, scale_y=1.0, scale_z=1.0,
            color_r=0.7, color_g=0.7, color_b=0.7, color_a=1.0
        )
        
        marker_array.markers.append(cylindre)
        marker_array.markers.append(saddle)
        self.marker_pub_.publish(marker_array)

    def create_stl_marker(self, id, mesh_path, pos_x, pos_y, pos_z,
                         ori_x, ori_y, ori_z, ori_w,
                         scale_x, scale_y, scale_z,
                         color_r, color_g, color_b, color_a):
        
        marker = Marker()
        
        # Header
        marker.header.frame_id = "map"
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
    node = welding_mesh_publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
