import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion


class PerceptionNode(Node):

    def __init__(self) -> None:
        super().__init__("perception_node")

        self.topicGraspPub = "/grasp_pose"
        self.graspLink = "camera_link"

        self.graspPub = self.create_publisher(PoseArray, self.topicGraspPub, 3)
        self.timer_ = self.create_timer(0.1, self.publish_grasp_pose)

        self.get_logger().info(f"Perception initialized.")

    def publish_grasp_pose(self):
        poseArrayMsg = PoseArray()
        poseArrayMsg.header.stamp = self.get_clock().now().to_msg()
        poseArrayMsg.header.frame_id = self.graspLink

        graspPoseMsg = Pose()
        graspPoseMsg.position = Point(x=0.285, y=-0.018, z=-0.095)
        graspPoseMsg.orientation = Quaternion(x=0.517, y=-0.499, z=0.495, w=-0.488)
        poseArrayMsg.poses.append(graspPoseMsg)

        preGraspPoseMsg = Pose()
        preGraspPoseMsg.position = Point(x=0.224, y=-0.018, z=-0.095)
        preGraspPoseMsg.orientation = Quaternion(
            x=0.517, y=-0.499, z=0.495, w=-0.488
        )
        poseArrayMsg.poses.append(preGraspPoseMsg)
        self.graspPub.publish(poseArrayMsg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
