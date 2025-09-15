import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
import time
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        # Moveit
        self._arm = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name="base_arm",
            end_effector_name="link6",              
            group_name="small_arm",
            use_move_group_action=True
        )
        self._gripper = MoveIt2(
            node=self,
            joint_names=["gripper_joint1"],
            base_link_name="base_arm",        
            group_name="gripper",
            end_effector_name="link6",     
            use_move_group_action=True
        )
        # Config
        srdf_package_dir = get_package_share_directory('small_arm_moveit_config')
        tree = ET.parse(srdf_package_dir + '/config/small_arm.srdf')
        root = tree.getroot()
        self._named_poses = {}
        for group_state in root.findall('group_state'):
            ## ignore when not small_arm group
            if group_state.attrib['group'] != 'small_arm' and group_state.attrib['group'] != 'gripper':
                continue
            named_pose = {'name':[], 'pose':[]}
            ## read joint
            for joint in group_state.findall('joint'):
                # print(joint.attrib)
                named_pose['name'].append(joint.attrib['name'])
                named_pose['pose'].append(float(joint.attrib['value']))
            self._named_poses[group_state.attrib['name']] = named_pose    
        # Finish
        self.get_logger().info("Ready")
   
    def goal_request_with_retry(self, pose: str | Pose) -> bool:
        retry = 0
        while not self.goal_request(pose):
            retry += 1
            self.get_logger().info(f"goal retry: {retry}")
            time.sleep(0.5)
            if retry >= 5:
                return False
        return True

    def goal_request(self, pose: str | Pose) -> bool:
        controller: MoveIt2 = None
        # Named pose
        if isinstance(pose, str):
            joint_state = self._named_poses[pose]
            if pose == "gripper_open" or pose == "gripper_close":
                self._gripper.move_to_configuration(joint_state['pose'], joint_state['name'])
                controller = self._gripper
            else:
                self._arm.move_to_configuration(joint_state['pose'], joint_state['name'])
                controller = self._arm
        # Normal pose
        else:
            self._arm.move_to_pose(
                position=pose.position,
                quat_xyzw=pose.orientation,
                frame_id="base_arm",             # Reference frame
                tolerance_position=0.001,         # Position tolerance
                tolerance_orientation=0.001,      # Orientation tolerance
                # cartesian=cartesian,
                # cartesian_max_step=cartesian_max_step,
                # cartesian_fraction_threshold=cartesian_fraction_threshold,
            )
            controller = self._arm
        # Wait until finish
        controller.wait_until_executed()
        error_code = controller.get_last_execution_error_code()
        if error_code.val != 1:
            self.get_logger().error(f"Execution error code: {error_code}")
            return False
        else:
            return True
