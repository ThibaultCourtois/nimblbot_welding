#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

CYLINDRE_PATH = "file://" + os.path.join(
    get_package_share_directory("welding_scene_publisher"),
    "scene_meshes",
    "Cylindre.stl",
)
SADDLE_PATH = "file://" + os.path.join(
    get_package_share_directory("welding_scene_publisher"),
    "scene_meshes",
    "SoudureSelleCheval.stl",
)
CUVE_PATH = "file://" + os.path.join(
    get_package_share_directory("welding_scene_publisher"), "scene_meshes", "Cuve.stl"
)

SCENES_CONFIG = {
    "standard": [
        {
            "path": CYLINDRE_PATH,
            "id": 0,
            "position": {"x": 1.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "scale": {"x": 0.001, "y": 0.001, "z": 0.001},
            "color": {"r": 0.7, "g": 0.7, "b": 0.7, "a": 1.0},
        },
        {
            "path": SADDLE_PATH,
            "id": 1,
            "position": {"x": 0.0, "y": 1.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071},
            "scale": {"x": 0.001, "y": 0.001, "z": 0.001},
            "color": {"r": 0.7, "g": 0.7, "b": 0.7, "a": 1.0},
        },
    ],
    "cuve": [
        {
            "path": CUVE_PATH,
            "id": 0,
            "position": {"x": 0.0, "y": 0.0, "z": 0.73},
            "orientation": {"x": 0.7070268, "y": 0.0, "z": 0.0, "w": 0.7071868},
            "scale": {"x": 0.005, "y": 0.005, "z": 0.005},
            "color": {"r": 0.5, "g": 0.5, "b": 0.5, "a": 1.0},
        }
    ],
}


class welding_mesh_publisher(Node):
    def __init__(self):
        super().__init__("welding_scene_publisher")
        self.declare_parameter("welding_scene", "standard")
        self.scene_type = (
            self.get_parameter("welding_scene").get_parameter_value().string_value
        )
        self.marker_pub_ = self.create_publisher(
            MarkerArray, "/welding_scene_publisher/welding_scene", 10
        )
        self.timer_ = self.create_timer(1.0, self.publish_scene)

    def publish_scene(self):
        marker_array = MarkerArray()

        if self.scene_type in SCENES_CONFIG:
            for mesh_config in SCENES_CONFIG[self.scene_type]:
                marker = self.create_stl_marker(mesh_config)
                marker_array.markers.append(marker)
        else:
            self.get_logger().warn(f"Scene type '{self.scene_type}' not found!")
        self.marker_pub_.publish(marker_array)

    def create_stl_marker(self, config):
        marker = Marker()

        # Header
        marker.header.frame_id = "nb/base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        # Marker properties
        marker.ns = "stl_objects"
        marker.id = config["id"]
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = config["position"]["x"]
        marker.pose.position.y = config["position"]["y"]
        marker.pose.position.z = config["position"]["z"]

        # Orientation
        marker.pose.orientation.x = config["orientation"]["x"]
        marker.pose.orientation.y = config["orientation"]["y"]
        marker.pose.orientation.z = config["orientation"]["z"]
        marker.pose.orientation.w = config["orientation"]["w"]

        # Scale
        marker.scale.x = config["scale"]["x"]
        marker.scale.y = config["scale"]["y"]
        marker.scale.z = config["scale"]["z"]

        # Color
        marker.color.r = config["color"]["r"]
        marker.color.g = config["color"]["g"]
        marker.color.b = config["color"]["b"]
        marker.color.a = config["color"]["a"]

        # STL file path
        marker.mesh_resource = config["path"]
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


if __name__ == "__main__":
    main()
