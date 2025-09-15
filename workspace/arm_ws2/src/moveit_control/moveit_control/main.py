import rclpy
from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from std_srvs.srv import Trigger  # Replace with your actual service
from std_srvs.srv import Trigger
from python_moveit_interface.srv import PoseRequest  # auto-generated from .srv
from pymoveit2 import MoveIt2, MoveIt2State
import time
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
# import threading

# joint_pose = [0.0, 1.6056, -0.2792, 0.0, -0.1919, 0.0]

srdf_package_dir = get_package_share_directory('small_arm_moveit_config')
tree = ET.parse(srdf_package_dir + '/config/small_arm.srdf')
root = tree.getroot()
named_poses = {}
for group_state in root.findall('group_state'):
    # print(group_state.attrib)
    ## ignore when not small_arm group
    if group_state.attrib['group'] != 'small_arm' and group_state.attrib['group'] != 'gripper':
        continue
    named_pose = {'name':[], 'pose':[]}
    ## read joint
    for joint in group_state.findall('joint'):
        # print(joint.attrib)
        named_pose['name'].append(joint.attrib['name'])
        named_pose['pose'].append(float(joint.attrib['value']))
    named_poses[group_state.attrib['name']] = named_pose    

class NamedGoalService(Node):
    def __init__(self):
        super().__init__('named_goal_service')
        self.moveit2 = None
        self.srv = self.create_service(PoseRequest, 'arm_goal_pose', self.handle_goal_request)
        self.finish_srv = self.create_service(Trigger, 'arm_goal_finish', self.handle_goal_finish)
        self.get_logger().info('Named goal node created, waiting for MoveIt2...')
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name="base_link",
            end_effector_name="link6",              
            group_name="small_arm",
            use_move_group_action=True
        )
        self.moveit2_grip = MoveIt2(
            node=self,
            joint_names=["gripper_joint1"],
            base_link_name="base_link",        
            group_name="gripper",
            end_effector_name="link6",     
            use_move_group_action=True
        )
        # self.moveit2.planning_time = 10.0
        # self.moveit2.max_velocity = 1.0
        # self.moveit2.max_acceleration = 1.0
        self.goal_pose_pending = None
        self.get_logger().info("MoveIt2 interface initialized.")

    def handle_goal_finish(self, request, response):
        status = self.moveit2.query_state()
        status_grip = self.moveit2_grip.query_state()
        if status != MoveIt2State.IDLE or status_grip != MoveIt2State.IDLE:
            response.success = False
            response.message = str(status)
            self.get_logger().info(f"Goal still running: {status}")
        else:
            response.success = True
            self.get_logger().info("Goal finish")
        return response


    def handle_goal_request(self, request, response):   
        target_name = request.message
        target_pose = request.target_pose
        try:
            if target_name != "":
                self.get_logger().info(f"Received named target: {target_name}")
                named_pose = named_poses[target_name]
                if target_name == "gripper_open" or target_name == "gripper_close":
                    self.moveit2_grip.move_to_configuration(named_pose['pose'], named_pose['name'])
                else:
                    self.moveit2.move_to_configuration(named_pose['pose'], named_pose['name'])
                # print(self.moveit2.joint_state)
                response.success = True
                response.message = f"Execute: {target_name}"
            else:
                self.get_logger().info(f"Received pose: {target_pose}")
                self.goal_pose_pending = target_pose
                #joint_state = self.moveit2.compute_ik(
                #    position=[target_pose.position.x, target_pose.position.y, target_pose.position.z],
                #    quat_xyzw=[target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
                #)
                #self.moveit2.move_to_configuration(joint_state.position, joint_state.name)
                response.success = True
                response.message = f"Execute: {target_pose}"
        except Exception as e:
            self.get_logger().error(f"MoveIt2 action failed: {e}")
            response.success = False
            response.message = f"Exception: {str(e)}"
        # self.moveit2.stop()
        # self.moveit2.clear_targets()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NamedGoalService()
    node.create_rate(10)
    while rclpy.ok():
        # node.get_logger().info("running")
        rclpy.spin_once(node)
        if node.goal_pose_pending is not None:
            target_pose = node.goal_pose_pending
            joint_state = node.moveit2.compute_ik(
                position=[target_pose.position.x, target_pose.position.y, target_pose.position.z],
                quat_xyzw=[target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
            )
            if joint_state:
                node.moveit2.move_to_configuration(joint_state.position, joint_state.name)
            node.goal_pose_pending = None
        if node.moveit2.query_state() != MoveIt2State.IDLE:
            node.get_logger().info("wait executed")
            node.moveit2.wait_until_executed()
        if node.moveit2_grip.query_state() != MoveIt2State.IDLE:
            node.get_logger().info("grip wait executed")
            node.moveit2_grip.wait_until_executed()
        time.sleep(0.1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
