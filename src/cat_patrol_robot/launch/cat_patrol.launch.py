# =============================================================================
# cat_patrol.launch.py — Bring up the entire cat patrol robot stack
# =============================================================================
#
# ROS 2 CONCEPT: LAUNCH FILES
#   A launch file starts MULTIPLE nodes at once and configures them.
#   Without a launch file, you'd need to open many terminals and run
#   each node manually.  Launch files automate this:
#     ros2 launch cat_patrol_robot cat_patrol.launch.py
#
#   ROS 2 launch files are Python scripts (not XML like ROS 1).
#   The function generate_launch_description() returns a LaunchDescription
#   — a list of actions (nodes to start, parameters to set, etc.).
#
# WHAT THIS LAUNCH FILE STARTS:
#   1. Astra camera node (color + depth streams)
#   2. Yahboom chassis driver (motor control, odometry, TF)
#   3. sllidar_ros2 driver (RPLidar /scan topic)  ← optional via start_lidar
#   4. patrol_node (C++ — the robot's brain)
#   5. mail_node (Python — sends email with photos)
#
# USAGE:
#   ros2 launch cat_patrol_robot cat_patrol.launch.py
#   ros2 launch cat_patrol_robot cat_patrol.launch.py robot:=none
#   ros2 launch cat_patrol_robot cat_patrol.launch.py robot:=r2
#   ros2 launch cat_patrol_robot cat_patrol.launch.py start_lidar:=false
#   ros2 launch cat_patrol_robot cat_patrol.launch.py lidar_serial_port:=/dev/ttyUSB1
#
# ARGUMENTS:
#   robot              — x3 (default), r2, or none (skip chassis driver)
#   use_sim_time       — false (default); set true for Gazebo simulation
#   chassis_serial_port — /dev/ttyUSB1 (verified Rosmaster CH340 1a86:7523).
#                         Identified by raw stream containing 0x7B...0x7D frames.
#   uvc_product_id     — 0x050f (Astra camera USB product ID)
#   start_lidar        — auto (default): true unless robot:=none. true/false to force.
#   lidar_serial_port  — by-id symlink for CP2102 → RPLidar A1 on /dev/ttyUSB0.
#                        Using by-id makes this immune to USB enumeration order.
#   lidar_baudrate     — 115200 (RPLidar A1 default).
#   lidar_frame_id     — laser (TF frame published by the driver).
# =============================================================================

"""Bring up Yahboom base + Astra camera + patrol + mail.

Robot variant: ``robot`` = x3 / r2 / none (drivers skipped for ``none``).
With ``robot:=none``, ``patrol_node`` stays idle unless you publish odom /
start patrol manually — no TF spam from a phantom robot.
Camera (astra_camera) is launched automatically with depth + color enabled.
"""
import os
import yaml

# ROS 2 LAUNCH CONCEPT: get_package_share_directory
#   Returns the INSTALLED share directory for a package.  This is where
#   colcon puts config files, launch files, etc. after building.
#   Example: get_package_share_directory('cat_patrol_robot')
#     → /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/cat_patrol_robot/share/cat_patrol_robot
from ament_index_python.packages import get_package_share_directory

# ROS 2 LAUNCH CONCEPT: LaunchDescription
#   The "container" for all launch actions.  You add nodes, arguments,
#   includes, etc. to it.  ROS 2 processes them in order.
from launch import LaunchDescription

# ROS 2 LAUNCH CONCEPT: LAUNCH ARGUMENTS (DeclareLaunchArgument)
#   Launch arguments are like command-line flags.  Users set them with:
#     ros2 launch package launch_file.py arg_name:=value
#   Inside the launch file, LaunchConfiguration('arg_name') retrieves the value.
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

# ROS 2 LAUNCH CONCEPT: INCLUDING OTHER LAUNCH FILES
#   IncludeLaunchDescription lets you nest launch files.  The camera driver
#   and chassis driver each have their OWN launch files — we just include them.
#   PythonLaunchDescriptionSource — for .py launch files
#   AnyLaunchDescriptionSource    — auto-detects .py, .xml, .yaml formats
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

# ROS 2 LAUNCH CONCEPT: LaunchConfiguration
#   A lazy reference to a launch argument's value.  It's NOT resolved when
#   the Python code runs — it's resolved LATER when the launch system
#   actually starts nodes.  This is why you can't just write:
#     if LaunchConfiguration('robot') == 'x3':  # WON'T WORK — it's not a string yet
#   Instead, use OpaqueFunction (see below) to access the resolved value.
from launch.substitutions import LaunchConfiguration

# ROS 2 LAUNCH CONCEPT: Node action
#   Tells the launch system to start a ROS 2 node.  You specify:
#   - package: which ROS package contains the executable
#   - executable: the program to run (defined in CMakeLists.txt or setup.py)
#   - name: the ROS node name (shows in `ros2 node list`)
#   - parameters: list of YAML files and/or dicts with parameter values
#   - output: 'screen' to see stdout/stderr in the terminal
from launch_ros.actions import Node
# Wrap string launch-arg values so a numeric param keeps its float type
# (a bare LaunchConfiguration reaches the node as a string and a double-typed
# parameter like trusted_center would raise a type mismatch).
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import LogInfo


def _read_odom_angular_scale(params_file: str, fallback: str = '0.5') -> str:
    """Read odom_angular_scale from YAML so launch defaults are config-driven."""
    try:
        with open(params_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = data.get('/**', {}).get('ros__parameters', {})
        value = params.get('odom_angular_scale', fallback)
        return str(value)
    except Exception:
        return fallback


def generate_launch_description():
    # Locate installed share directories for each package we depend on
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_bringup = get_package_share_directory('yahboomcar_bringup')
    pkg_astra = get_package_share_directory('astra_camera')

    # Path to our YAML parameter file
    params_file = os.path.join(pkg_cat, 'config', 'cat_patrol_params.yaml')
    odom_angular_scale_default = _read_odom_angular_scale(params_file, '0.5')

    # -----------------------------------------------------------------------
    # Declare launch arguments (these become command-line options)
    # -----------------------------------------------------------------------
    # ROS 2 LAUNCH CONCEPT: DeclareLaunchArgument
    #   Registers an argument so it can be set from the command line:
    #     ros2 launch cat_patrol_robot cat_patrol.launch.py robot:=r2
    #   'choices' restricts valid values (optional but helps catch typos).
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='x3',
        description='Yahboom chassis: x3, r2, or none (skip driver launch)',
        choices=['x3', 'r2', 'none'],
    )

    use_sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # =========================================================================
    # SERIAL PORT MAPPING for this Jetson (verified empirically on 2026-05-19):
    #   /dev/ttyUSB0 -> RPLidar A1            (Silicon Labs CP2102, 10c4:ea60)
    #     Stable by-id symlink:
    #     /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
    #
    #   /dev/ttyUSB1 -> Yahboom Rosmaster MCU (CH340 1a86:7523)
    #     Identification: raw stream contains repeating 0x7B ... 0x7D frames
    #     (Yahboom telemetry protocol). NO by-id symlink: this CH340 variant
    #     doesn't expose a serial number, so it stays as /dev/ttyUSB1.
    #
    #   /dev/ttyUSB2 -> UNVERIFIED peripheral (CH340 1a86:7522)
    #     Stable by-id symlink:
    #     /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
    #     Observed: silent on idle (no continuous byte stream at 115200).
    #     Possible candidates: Orbbec Astra Pro UART control endpoint,
    #     joystick, IMU, or other peripheral. Not used by cat_patrol_robot —
    #     identify via the unplug test if you need to know.
    #
    # Override these args at launch time if USB enumeration order shifts.
    # =========================================================================
    chassis_serial_arg = DeclareLaunchArgument(
        'chassis_serial_port',
        default_value='/dev/ttyUSB1',
        description='Yahboom Rosmaster MCU serial device. Verified by raw-byte sniff: '
                    'the chassis port streams 0x7B...0x7D framed telemetry. '
                    'No stable by-id symlink (CH340 1a86:7523 has no serial number).',
    )

    odom_angular_scale_arg = DeclareLaunchArgument(
        'odom_angular_scale',
        default_value=odom_angular_scale_default,
        description='base_node_X3 angular_scale for odometry yaw calibration',
    )

    uvc_product_id_arg = DeclareLaunchArgument(
        'uvc_product_id',
        default_value='0x050f',
        description='Astra camera UVC product ID',
    )

    # --- LiDAR arguments -----------------------------------------------------
    # Use the by-id-stable port for the RPLidar.  We intentionally avoid
    # /dev/rplidar because the current udev rule on this Jetson points it
    # at the Yahboom chassis MCU (Silicon Labs CP210x, 10c4:ea60), which
    # would clash with Mcnamu_driver_X3.  The actual LiDAR enumerates as
    # /dev/ttyUSB2 (1a86 CH340) on this hardware.
    start_lidar_arg = DeclareLaunchArgument(
        'start_lidar',
        default_value='auto',
        description="Start sllidar_ros2 driver. 'true' / 'false' / 'auto' "
                    "(auto = true unless robot:=none).",
        choices=['true', 'false', 'auto'],
    )
    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ),
        description='Serial device for the RPLidar A1. Uses the by-id symlink so '
                    'USB enumeration order does not break the launch. Resolves to '
                    '/dev/ttyUSB0 on this Jetson. (NOTE: do NOT use /dev/rplidar — '
                    'that udev symlink races with hot-plug order.)',
    )
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='RPLidar baudrate (115200 for A1 / A2-2.x, 256000 for A2-3.x / A3, 1000000 for S1).',
    )
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser_link',
        description='TF frame_id published by the LiDAR driver. Must match the URDF '
                    'static TF chain (yahboomcar_X3.urdf defines base_link -> laser_link). '
                    'If this is set to a frame that does not exist in /tf_static '
                    '(e.g. plain "laser"), then SLAM/AMCL/Nav2 will silently fail to '
                    'project /scan into base_link and produce empty maps.',
    )

    # --- Brain / mail toggles -----------------------------------------------
    # The unified patrol_system.launch.py reuses this file for SENSORS + BRINGUP
    # only, driving the robot with the Nav2 patrol_manager instead of the odom
    # patrol_node. Set start_patrol_node:=false there so two brains never fight
    # over /cmd_vel. mail_node is shared (both brains publish mail requests to
    # the same topic), so it defaults on and can be disabled if launched elsewhere.
    start_patrol_node_arg = DeclareLaunchArgument(
        'start_patrol_node',
        default_value='true',
        description='Launch the odom-based patrol_node (the robots brain). '
                    'Set false when the Nav2 patrol_manager is the brain.',
        choices=['true', 'false'],
    )
    start_mail_arg = DeclareLaunchArgument(
        'start_mail',
        default_value='true',
        description='Launch the Python mail_node (SMTP sender).',
        choices=['true', 'false'],
    )

    # --- Scan front filter (/scan -> /scan_filtered) -------------------------
    # These defaults match myscripts2/t2.5.sh: the RPLidar is mounted ~180 deg
    # vs the URDF, so the trusted front arc is centered on pi.
    scan_trusted_center_arg = DeclareLaunchArgument(
        'scan_trusted_center',
        default_value='3.141592653589793',
        description='Bearing (rad) the trusted laser arc is centered on '
                    '(pi = real front, given the 180 deg lidar mount).',
    )
    scan_trusted_halfwidth_arg = DeclareLaunchArgument(
        'scan_trusted_halfwidth',
        default_value='1.92',
        description='Half-width (rad) of the trusted arc (~110 deg keeps the '
                    'clean front 220 deg, drops the rear antenna/wire cone).',
    )

    # -----------------------------------------------------------------------
    # OpaqueFunction: resolve arguments → build node list
    # -----------------------------------------------------------------------
    # ROS 2 LAUNCH CONCEPT: OpaqueFunction
    #   Launch arguments (LaunchConfiguration) are "lazy" — they're not
    #   Python strings until the launch system runs.  But sometimes you need
    #   to make DECISIONS based on argument values (if robot == 'x3': ...).
    #
    #   OpaqueFunction solves this: it wraps a regular Python function that
    #   receives a "context" object.  Inside this function, you can call
    #     LaunchConfiguration('robot').perform(context)
    #   to get the ACTUAL string value, and then use normal if/else logic.
    #
    #   The function must return a list of launch actions (nodes, includes, etc.).
    def compose(context):  # noqa: D401
        """OpaqueFunction: resolved robot string → correct bringup + patrol params."""

        entities = []  # Collect all launch actions here

        # Loud banner so the operator can verify exactly which YAML and which
        # capture_completion_extra_deg / odom_angular_scale are about to be used.
        extra_deg = 'unknown'
        try:
            with open(params_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            params = data.get('/**', {}).get('ros__parameters', {})
            extra_deg = params.get('capture_completion_extra_deg', 'unset')
        except Exception as ex:
            extra_deg = f'read-error: {ex}'

        entities.append(LogInfo(msg='=========== CAT PATROL LAUNCH RESOLVED ==========='))
        entities.append(LogInfo(msg=f'  pkg_share        : {pkg_cat}'))
        entities.append(LogInfo(msg=f'  params_file      : {params_file}'))
        entities.append(LogInfo(msg=f'  odom_angular_scale (default from yaml) : {odom_angular_scale_default}'))
        entities.append(LogInfo(msg=f'  capture_completion_extra_deg (yaml)    : {extra_deg}'))
        entities.append(LogInfo(msg='=================================================='))

        # Resolve the 'robot' argument to an actual string
        robot = LaunchConfiguration('robot').perform(context)
        robot = robot.strip().lower()

        # --- Astra camera (always launched regardless of robot type) ---
        # This starts the camera driver which publishes:
        #   /camera/color/image_raw  (color frames)
        #   /camera/depth/image_raw  (depth frames)
        entities.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(pkg_astra, 'launch', 'astra_pro.launch.xml')),
                launch_arguments=[
                    ('uvc_product_id', LaunchConfiguration('uvc_product_id')),
                ],
            ))

        # --- Chassis driver (conditional on robot type) ---
        # The bringup launch file starts a small stack:
        #   - Mcnamu_driver_X3      (Python)  Reads MCU -> /vel_raw, /imu/data_raw,
        #                                     /joint_states, /voltage. Listens on
        #                                     /cmd_vel. Does NOT publish /odom or TF.
        #   - base_node_X3          (C++)     Integrates wheel velocity into /odom_raw.
        #                                     Does NOT publish odom->base_footprint TF
        #                                     (pub_odom_tf=false), because…
        #   - imu_filter_madgwick   (C++)     /imu/data_raw -> /imu/data (filtered).
        #   - robot_localization EKF (C++)    Fuses /odom_raw + /imu/data ->
        #                                     /odom (filtered) AND publishes the
        #                                     odom -> base_footprint TF.
        # If you ever see Nav2 "stuck" because the TF chain is broken, the EKF
        # is the most likely culprit -- not the chassis driver itself.
        if robot == 'x3':
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_bringup, 'launch', 'yahboomcar_bringup_X3_launch.py')),
                    launch_arguments=[
                        ('chassis_serial_port', LaunchConfiguration('chassis_serial_port')),
                        ('angular_scale', LaunchConfiguration('odom_angular_scale')),
                        ('use_joystick', 'false'),
                    ],
                ))
        elif robot == 'r2':
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_bringup, 'launch', 'yahboomcar_bringup_R2_launch.py')),
                ))
        elif robot != 'none':
            raise RuntimeError(
                'Invalid robot=%r — use robot:=x3, robot:=r2, or robot:=none' % (robot, ))

        # robot:=none → don't start patrolling (no TF, no motors)
        start_patrol = robot != 'none'

        # --- LiDAR driver (sllidar_ros2 — publishes /scan) -------------------
        # We resolve 'start_lidar' here so we can apply the "auto" default
        # (auto → true unless robot:=none, where there's typically no robot
        # hardware connected at all).
        start_lidar_val = LaunchConfiguration('start_lidar').perform(context).strip().lower()
        if start_lidar_val == 'auto':
            start_lidar_resolved = (robot != 'none')
        elif start_lidar_val == 'true':
            start_lidar_resolved = True
        elif start_lidar_val == 'false':
            start_lidar_resolved = False
        else:
            raise RuntimeError(
                'Invalid start_lidar=%r — use true / false / auto' % (start_lidar_val,))

        if start_lidar_resolved:
            try:
                pkg_sllidar = get_package_share_directory('sllidar_ros2')
            except Exception as ex:  # noqa: BLE001
                raise RuntimeError(
                    'start_lidar requested but sllidar_ros2 package not found. '
                    'Either install it, source the overlay that contains it, '
                    'or pass start_lidar:=false. (%s)' % (ex,))
            entities.append(LogInfo(msg=(
                f'[cat_patrol] Starting sllidar_ros2 driver on '
                f'{LaunchConfiguration("lidar_serial_port").perform(context)} '
                f'@ {LaunchConfiguration("lidar_baudrate").perform(context)} baud')))
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_sllidar, 'launch', 'sllidar_launch.py')),
                    launch_arguments=[
                        ('serial_port', LaunchConfiguration('lidar_serial_port')),
                        ('serial_baudrate', LaunchConfiguration('lidar_baudrate')),
                        ('frame_id', LaunchConfiguration('lidar_frame_id')),
                    ],
                ))

            # --- Scan front filter (/scan -> /scan_filtered) ------------------
            # AMCL and both Nav2 costmaps consume /scan_filtered, NOT /scan.
            # The RPLidar A1 is mounted ~180 deg vs the URDF, so we KEEP the
            # trusted front arc centered on pi (trusted_center) with half-width
            # 1.92 rad (~110 deg) and drop the rear cone (antennas/wires that
            # move with the robot and wreck scan matching). Matches
            # myscripts2/t2.5.sh — WITHOUT this node, localization + obstacle
            # avoidance get no laser at all.
            entities.append(Node(
                package='yahboomcar_bringup',
                executable='scan_front_filter',
                name='scan_front_filter',
                output='screen',
                parameters=[{
                    'input_topic': '/scan',
                    'output_topic': '/scan_filtered',
                    'trusted_center': ParameterValue(
                        LaunchConfiguration('scan_trusted_center'), value_type=float),
                    'trusted_halfwidth': ParameterValue(
                        LaunchConfiguration('scan_trusted_halfwidth'), value_type=float),
                }],
            ))
        else:
            entities.append(LogInfo(msg='[cat_patrol] LiDAR driver skipped (start_lidar=false)'))

        # --- Patrol node (C++, the robot's brain) ---
        # ROS 2 CONCEPT: PARAMETER MERGING
        #   The 'parameters' list can contain YAML files AND Python dicts.
        #   Values from later entries OVERRIDE earlier ones.  So params_file
        #   provides all defaults, and the dict overrides use_sim_time and
        #   start_patrol_on_boot specifically.
        start_patrol_node = LaunchConfiguration('start_patrol_node').perform(context).strip().lower() == 'true'
        if start_patrol_node:
            patrol_node = Node(
                package='cat_patrol_robot',
                executable='patrol_node',
                name='patrol_node',
                output='screen',
                parameters=[
                    params_file,                    # Load all defaults from YAML
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'start_patrol_on_boot': start_patrol,
                    },
                ],
            )
            entities.append(patrol_node)
        else:
            entities.append(LogInfo(
                msg='[cat_patrol] start_patrol_node=false — odom patrol_node skipped '
                    '(Nav2 patrol_manager is the brain).'))

        # --- Mail node (Python, sends email with photos) ---
        # This node sits idle until a brain publishes a mail request.
        # SMTP credentials must be set as environment variables.
        start_mail = LaunchConfiguration('start_mail').perform(context).strip().lower() == 'true'
        if start_mail:
            entities.append(
                Node(
                    package='cat_patrol_robot',
                    executable='mail_node',
                    name='mail_node',
                    output='screen',
                    parameters=[{'mail_request_topic': '/cat_patrol/mail_request'}],
                ))

        return entities

    # -----------------------------------------------------------------------
    # Assemble the LaunchDescription
    # -----------------------------------------------------------------------
    # ROS 2 LAUNCH CONCEPT: ACTION ORDER
    #   Actions in the list are processed in order:
    #     1. Declare all arguments first (so they're available to later actions)
    #     2. Then the OpaqueFunction which creates and returns nodes
    return LaunchDescription([
        robot_arg,
        use_sim_arg,
        chassis_serial_arg,
        odom_angular_scale_arg,
        uvc_product_id_arg,
        start_lidar_arg,
        lidar_serial_port_arg,
        lidar_baudrate_arg,
        lidar_frame_id_arg,
        start_patrol_node_arg,
        start_mail_arg,
        scan_trusted_center_arg,
        scan_trusted_halfwidth_arg,
        OpaqueFunction(function=compose),
    ])
