"""Launch the GP7 assignment scene with Gazebo-native joint control."""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from gp7robot_description.kinematics import (
    JOINT_NAMES,
    UPRIGHT_REFERENCE_CONFIGURATION,
    forward_kinematics,
)


def _add_inertial(link, mass, inertia):
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(
        inertial,
        "inertia",
        ixx=str(inertia),
        ixy="0",
        ixz="0",
        iyy=str(inertia),
        iyz="0",
        izz=str(inertia),
    )


def _add_finger_geometry(link, name):
    visual = ET.SubElement(link, "visual", name=f"{name}_visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", size="0.010 0.050 0.100")
    material = ET.SubElement(visual, "material", name="gp7_gripper_dark")
    ET.SubElement(material, "color", rgba="0.18 0.20 0.23 1")

    collision = ET.SubElement(link, "collision", name=f"{name}_collision")
    ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "box", size="0.010 0.050 0.100")


def _add_fixed_joint(root, name, parent, child, xyz):
    joint = ET.SubElement(root, "joint", name=name, type="fixed")
    ET.SubElement(joint, "origin", xyz=xyz, rpy="0 0 0")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)


def _rpy_from_matrix(rotation):
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
    return roll, pitch, yaw


def _simulation_robot_description(urdf_path):
    """Remove the incompatible ros2_control plugin and add Gazebo systems."""
    root = ET.parse(urdf_path).getroot()
    for child in list(root):
        if child.tag in ("ros2_control", "gazebo"):
            root.remove(child)

    world_link = ET.Element("link", name="world")
    root.insert(0, world_link)
    world_anchor = ET.SubElement(root, "joint", name="gp7_world_fixed", type="fixed")
    ET.SubElement(world_anchor, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(world_anchor, "parent", link="world")
    ET.SubElement(world_anchor, "child", link="base_link")
    world_reference = ET.SubElement(root, "gazebo", reference="world")
    ET.SubElement(world_reference, "static").text = "true"

    gripper = ET.SubElement(root, "link", name="gp7_gripper")
    _add_inertial(gripper, 0.12, 0.00008)
    link6_rotation = forward_kinematics(UPRIGHT_REFERENCE_CONFIGURATION)[:3, :3]
    mount_rotation = link6_rotation.T
    mount_translation = mount_rotation @ np.array([0.0, 0.0, -0.075])
    mount_rpy = _rpy_from_matrix(mount_rotation)
    gripper_mount = ET.SubElement(root, "joint", name="gp7_gripper_mount", type="fixed")
    ET.SubElement(
        gripper_mount,
        "origin",
        xyz=" ".join(f"{value:.12f}" for value in mount_translation),
        rpy=" ".join(f"{value:.12f}" for value in mount_rpy),
    )
    ET.SubElement(gripper_mount, "parent", link="Link6")
    ET.SubElement(gripper_mount, "child", link="gp7_gripper")

    for name, offset in (("gp7_left_finger", "-0.0325 0 0.025"), ("gp7_right_finger", "0.0375 0 0.025")):
        finger = ET.SubElement(root, "link", name=name)
        _add_inertial(finger, 0.05, 0.00005)
        _add_finger_geometry(finger, name)
        _add_fixed_joint(root, f"{name}_mount", "gp7_gripper", name, offset)

    gazebo = ET.SubElement(root, "gazebo")
    # Enable Gazebo physics contacts between non-adjacent links in this model.
    ET.SubElement(gazebo, "self_collide").text = "true"
    trajectory_plugin = ET.SubElement(
        gazebo,
        "plugin",
        filename="gz-sim-joint-trajectory-controller-system",
        name="gz::sim::systems::JointTrajectoryController",
    )
    p_gains = (180, 180, 150, 35, 25, 15)
    d_gains = (28, 28, 24, 7, 5, 3)
    # Integral action removes the steady-state gravity sag of a pure PD loop;
    # the clamp (in N m) bounds wind-up during long moves.
    # Only gravity-loaded joints (2, 3, 5) get integral action; joint 1 has a
    # vertical axis and would only overshoot from wind-up.
    i_gains = (0, 120, 100, 0, 8, 0)
    i_clamps = (1, 60, 45, 1, 6, 1)
    torque_limits = (260, 260, 220, 80, 55, 35)
    for joint, p_gain, i_gain, d_gain, i_clamp, torque in zip(
        JOINT_NAMES, p_gains, i_gains, d_gains, i_clamps, torque_limits
    ):
        ET.SubElement(trajectory_plugin, "joint_name").text = joint
        ET.SubElement(trajectory_plugin, "initial_position").text = "0"
        ET.SubElement(trajectory_plugin, "position_p_gain").text = str(p_gain)
        ET.SubElement(trajectory_plugin, "position_i_gain").text = str(i_gain)
        ET.SubElement(trajectory_plugin, "position_d_gain").text = str(d_gain)
        ET.SubElement(trajectory_plugin, "position_i_min").text = str(-i_clamp)
        ET.SubElement(trajectory_plugin, "position_i_max").text = str(i_clamp)
        ET.SubElement(trajectory_plugin, "position_cmd_min").text = str(-torque)
        ET.SubElement(trajectory_plugin, "position_cmd_max").text = str(torque)

    ET.SubElement(
        gazebo,
        "plugin",
        filename="gz-sim-joint-state-publisher-system",
        name="gz::sim::systems::JointStatePublisher",
    )
    detachable = ET.SubElement(
        gazebo,
        "plugin",
        filename="gz-sim-detachable-joint-system",
        name="gz::sim::systems::DetachableJoint",
    )
    # URDF-to-SDF conversion may merge fixed joints. Link6 is always preserved,
    # and the detachable system records the current relative grasp transform.
    
    ET.SubElement(detachable, "parent_link").text = "Link6"
    ET.SubElement(detachable, "child_model").text = "BoxB"
    ET.SubElement(detachable, "child_link").text = "link"
    ET.SubElement(detachable, "attach_topic").text = "/gp7/gripper/attach"
    ET.SubElement(detachable, "detach_topic").text = "/gp7/gripper/detach"
    

    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    package_path = get_package_share_directory("gp7robot_description")
    urdf_path = os.path.join(package_path, "urdf", "GP7.urdf")
    default_world = os.path.join(package_path, "worlds", "gp7_pick_place_reachable.sdf")
    robot_description = _simulation_robot_description(urdf_path)

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="World SDF; choose gp7_task_spec.sdf to load the exact assignment layout",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run the Gazebo server only (no GUI)",
    )
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "2", LaunchConfiguration("world")],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v", "2", LaunchConfiguration("world")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/GP7Robot/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
            "/gp7/gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/gp7/gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/world/gp7_pick_place/model/GP7Robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[("/world/gp7_pick_place/model/GP7Robot/joint_state", "/joint_states")],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "GP7Robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
        output="screen",
    )

    pick_place = Node(
        package="gp7robot_description",
        executable="gp7_pick_place",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_demo")),
    )
    run_demo_arg = DeclareLaunchArgument(
        "run_demo",
        default_value="true",
        description="Start the pick-and-place demo automatically",
    )

    resource_paths = [os.path.dirname(package_path)]
    existing_resource_paths = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource_paths:
        resource_paths.append(existing_resource_paths)
    gazebo_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(resource_paths),
    )

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            run_demo_arg,
            gazebo_resources,
            robot_state_publisher,
            gazebo,
            gazebo_headless,
            bridge,
            TimerAction(period=5.0, actions=[spawn_robot]),
            TimerAction(period=10.0, actions=[pick_place]),
        ]
    )
