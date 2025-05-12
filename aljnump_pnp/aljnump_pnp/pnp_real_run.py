import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RCLDuration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from onrobotsg_interfaces.srv import OnrobotSG
from rcl_interfaces.srv import GetParameters
import time
from .kdl_kinematic import KDLChain
from .kdl_urdf_parser import treeFromString
import tf2_geometry_msgs


class UR5ePnP(Node):

    def __init__(self):
        super().__init__("pnp_real")
        self.cbg = ReentrantCallbackGroup()
        self.cbgtimer = ReentrantCallbackGroup()

        self.declare_parameter("chain.root", rclpy.Parameter.Type.STRING)
        self.declare_parameter("chain.tip", rclpy.Parameter.Type.STRING)
        self.declare_parameter("chain.camera", rclpy.Parameter.Type.STRING)

        self.declare_parameter("gripper.gripper_open", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter(
            "gripper.gripper_close", rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            "gripper.command_service", rclpy.Parameter.Type.STRING
        )
        self.declare_parameter(
            "gripper.grasp_pose_topic", rclpy.Parameter.Type.STRING
        )

        self.declare_parameter("joint.name", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("joint.dropj", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint.homej", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint.state_topic", rclpy.Parameter.Type.STRING)
        self.declare_parameter("joint.command_action", rclpy.Parameter.Type.STRING)

        self.root = (
            self.get_parameter("chain.root").get_parameter_value().string_value
        )
        self.tip = (
            self.get_parameter("chain.tip").get_parameter_value().string_value
        )
        self.camera = (
            self.get_parameter("chain.camera").get_parameter_value().string_value
        )
        self.gripper_open_value = (
            self.get_parameter("gripper.gripper_open")
            .get_parameter_value()
            .double_value
        )
        self.gripper_close_value = (
            self.get_parameter("gripper.gripper_close")
            .get_parameter_value()
            .double_value
        )
        self.command_service = (
            self.get_parameter("gripper.command_service")
            .get_parameter_value()
            .string_value
        )
        self.graspPoseTopic = (
            self.get_parameter("gripper.grasp_pose_topic")
            .get_parameter_value()
            .string_value
        )
        self.jntName = (
            self.get_parameter("joint.name")
            .get_parameter_value()
            .string_array_value
        )
        self.dropj = (
            self.get_parameter("joint.dropj")
            .get_parameter_value()
            .double_array_value.tolist()
        )
        self.homej = (
            self.get_parameter("joint.homej")
            .get_parameter_value()
            .double_array_value.tolist()
        )
        self.jntStateTopic = (
            self.get_parameter("joint.state_topic")
            .get_parameter_value()
            .string_value
        )
        self.jntCommandAction = (
            self.get_parameter("joint.command_action")
            .get_parameter_value()
            .string_value
        )

        # ik solver
        self.kdl_chain = KDLChain()
        self.urdf_cli = self.create_client(
            GetParameters, "robot_state_publisher/get_parameters"
        )
        self.urdf_cli.wait_for_service()
        self.urdf_req = GetParameters.Request()
        self.urdf_req.names = ["robot_description"]
        self.urdf_cli.call_async(self.urdf_req).add_done_callback(
            self.urdf_callback
        )

        # robot controller
        self.jntcsub = self.create_subscription(
            JointState,
            self.jntStateTopic,
            self.jntc_cb,
            2,
            callback_group=self.cbg,
        )
        self.trajAct = ActionClient(
            self,
            FollowJointTrajectory,
            self.jntCommandAction,
            callback_group=self.cbg,
        )
        self.jntc = [0.0] * 6

        # traformation
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # gripper
        self.grprcli = self.create_client(OnrobotSG, self.command_service)

        # grasp pose
        self.graspSub = self.create_subscription(
            PoseArray,
            self.graspPoseTopic,
            self.grasp_callback,
            1,
            callback_group=self.cbg,
        )
        self.graspPoseCamlink = None
        self.preGraspPoseCamlink = None

        self.timer_ = self.create_timer(3, self.cb, callback_group=self.cbgtimer)
        self.isReady = True

    def cb(self) -> None:
        self.timer_.cancel()

        # home
        self.action_forward(self.homej, t=[2])
        time.sleep(1)
        self.get_logger().info("Moving to home")
        while self.isReady is False:
            pass

        # open gripp
        self.send_request_gripper(self.gripper_open_value)
        time.sleep(2)

        # compute grasp pose
        Hgrasp, Hpregrasp = self.get_grasp_and_pregrasp()
        Jntgrasp = self.handle_ik_pose(Hgrasp)
        Jntpregrasp = self.handle_ik_pose(Hpregrasp)

        # pregrasp
        self.action_forward(Jntpregrasp, t=[10])
        time.sleep(1)
        self.get_logger().info("Moving to pregrasp")
        while self.isReady is False:
            pass

        # grasp
        self.action_forward(Jntgrasp, t=[5])
        time.sleep(1)
        self.get_logger().info("Moving to grasp")
        while self.isReady is False:
            pass

        # close gripp
        self.send_request_gripper(self.gripper_close_value)
        time.sleep(2)

        # pregrasp
        self.action_forward(Jntpregrasp, t=[3])
        time.sleep(1)
        self.get_logger().info("Moving to pregrasp")
        while self.isReady is False:
            pass

        # home
        self.action_forward(self.homej, t=[7])
        time.sleep(1)
        self.get_logger().info("Moving to home")
        while self.isReady is False:
            pass

        # drop
        self.action_forward(self.dropj, t=[10])
        time.sleep(1)
        self.get_logger().info("Moving to drop off")
        while self.isReady is False:
            pass

        # open gripp
        self.send_request_gripper(self.gripper_open_value)
        time.sleep(2)

        # home
        self.action_forward(self.homej, t=[10])
        time.sleep(1)
        self.get_logger().info("Moving to home")
        while self.isReady is False:
            pass

        self.timer_.reset()

    def action_forward(self, path, velo=None, acc=None, t=None):
        # (dof, seq), t
        if isinstance(path, list):
            path = np.array(path).reshape(6, -1)
        if velo is None:
            velo = np.zeros_like(path)
        if acc is None:
            acc = np.zeros_like(path)

        goalMsg = FollowJointTrajectory.Goal()
        goalMsg.trajectory = JointTrajectory()
        goalMsg.trajectory.joint_names = self.jntName
        for i in range(path.shape[1]):
            point = JointTrajectoryPoint()
            point.positions = path[:, i].tolist()
            point.velocities = velo[:, i].tolist()
            point.accelerations = acc[:, i].tolist()
            point.time_from_start = RCLDuration(seconds=t[i]).to_msg()
            goalMsg.trajectory.points.append(point)

        self.trajAct.wait_for_server()
        self.sendGoalFuture = self.trajAct.send_goal_async(
            goalMsg, feedback_callback=self.feedback_callback
        )
        self.sendGoalFuture.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedbackMsg):
        feedback = feedbackMsg.feedback.error.positions
        error = np.linalg.norm(feedback)
        # self.get_logger().info(f"Error value : {error}")

    def goal_response_callback(self, future):
        goalHandle = future.result()
        if not goalHandle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self.isReady = False

        self.getResultFuture = goalHandle.get_result_async()
        self.getResultFuture.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Goal succeeded!")
            self.isReady = True
        else:
            self.get_logger().info(
                f"Goal failed with status: {result.error_string}!"
            )

    def jntc_cb(self, msg):
        self.jntc = msg.position

    def urdf_callback(self, future):
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Failed to get URDF model.")
                return
            urdf_model = response.values[0].string_value
            self.get_logger().info("URDF model received.")
            ok, kdl_chain = treeFromString(urdf_model)
            self.kdl_chain.config_kdl_kinemtic(kdl_chain, self.root, self.tip)
            self.get_logger().info("KDL chain configured.")
        except Exception as e:
            self.get_logger().error(f"Error in URDF callback: {e}")

    def handle_ik_pose(self, pose: Pose):
        target_kdl = self.kdl_chain.make_frame(
            [pose.position.x, pose.position.y, pose.position.z],
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        )

        Jntinit = self.kdl_chain.make_joint_array(self.jntc)
        joint_positions = self.kdl_chain.inverse_kinematics(target_kdl, Jntinit)
        Jntik = self.kdl_chain.make_joint_list(joint_positions)
        return Jntik

    def send_request_gripper(self, width):
        reqMsg = OnrobotSG.Request()
        reqMsg.desiredwidth = width
        future = self.grprcli.call_async(reqMsg)

    def grasp_callback(self, msg):
        # Index 0 is grasp pose, index 1 is pregrasp pose
        self.graspPoseCamlink = msg.poses[0]
        self.preGraspPoseCamlink = msg.poses[1]

    def get_transformation(self, toFrameRel, fromFrameRel):
        while True:
            try:
                now = rclpy.time.Time()
                transformstamped = self.tfBuffer.lookup_transform(
                    toFrameRel, fromFrameRel, now
                )
                return transformstamped
            except:
                self.get_logger().info("Waiting for transformation")

    def get_grasp_and_pregrasp(self):
        Hcamtobase = self.get_transformation(self.root, self.camera)

        while True:
            if self.graspPoseCamlink is not None:
                # Hgrasptobase = Hcamtobase @ Hgrasptocam
                # Hpregrasptobase = Hcamtobase @ Hpregrasptocam

                Hgrasptobase = tf2_geometry_msgs.do_transform_pose(
                    pose=self.graspPoseCamlink, transform=Hcamtobase
                )
                Hpregrasptobase = tf2_geometry_msgs.do_transform_pose(
                    pose=self.preGraspPoseCamlink, transform=Hcamtobase
                )
                return (
                    Hgrasptobase,
                    Hpregrasptobase,
                )


def main(args=None):
    rclpy.init(args=args)
    node = UR5ePnP()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        node.destroy_node()
        exe.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
