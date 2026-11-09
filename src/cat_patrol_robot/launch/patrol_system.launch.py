# =============================================================================
# patrol_system.launch.py — THE whole cat-patrol stack in one launch file
# =============================================================================
#
# Brings up every piece needed for an autonomous Nav2 waypoint patrol, in the
# right order, so you never have to run N launch files by hand again:
#
#   1. Sensors + bringup  (cat_patrol.launch.py, start_patrol_node:=false)
#   2. Localization       (localization.launch.py) — AMCL@home
#   3. Navigation         (navigation.launch.py)
#   4. Patrol manager     (patrol_manager.launch.py) — waypoint cycle brain
#   5. Detector + voice   (cat_voice.launch.py)
#
# Audio: when use_voice:=true, connect_usb_speaker starts early (~1s) so the
# USB voice card becomes Pulse default sink during bringup. cat_voice then
# plays ready-chime.mp3 and latches /cat_patrol/audio_ready; patrol_manager
# stays in WAKING until that (require_audio_ready:=true) — fail-closed, no
# drive until audio is proven. (Legacy BT path: start_bt via cat_voice only.)
#
# USAGE (normally via the `patrol` CLI, but works standalone too):
#   ros2 launch cat_patrol_robot patrol_system.launch.py
#   ros2 launch cat_patrol_robot patrol_system.launch.py use_rviz:=true
#   ros2 launch cat_patrol_robot patrol_system.launch.py use_voice:=false
# =============================================================================
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, rel_launch, launch_arguments=None):
    path = os.path.join(get_package_share_directory(pkg), 'launch', rel_launch)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=launch_arguments or [],
    )


def compose(context):
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_pm = get_package_share_directory('patrol_manager')

    robot = LaunchConfiguration('robot').perform(context)
    start_lidar = LaunchConfiguration('start_lidar').perform(context)
    use_detector = LaunchConfiguration('use_detector').perform(context).strip().lower() == 'true'
    use_voice = LaunchConfiguration('use_voice').perform(context).strip().lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).strip().lower() == 'true'

    entities = [
        LogInfo(msg='=========== PATROL SYSTEM (Nav2 brain) ==========='),
        LogInfo(msg=f'  robot={robot} start_lidar={start_lidar} '
                    f'use_detector={use_detector} use_voice={use_voice} use_rviz={use_rviz}'),
        LogInfo(msg='=================================================='),
    ]

    # 1) Sensors + bringup + mail, WITHOUT the odom patrol_node.
    entities.append(_include(
        'cat_patrol_robot', 'cat_patrol.launch.py',
        launch_arguments=[
            ('robot', robot),
            ('start_lidar', start_lidar),
            ('start_patrol_node', 'false'),
            ('start_mail', 'true'),
            ('chassis_serial_port', '/dev/myserial'),
        ],
    ))

    # 1b) USB voice card early (overlaps AMCL/Nav2). Skip chime here — cat_voice
    # owns the single ready-chime probe + /cat_patrol/audio_ready publish.
    if use_voice:
        usb_script = os.path.join(
            get_package_prefix('cat_patrol_robot'), 'lib', 'cat_patrol_robot',
            'connect_usb_speaker.sh')
        entities.append(TimerAction(period=1.0, actions=[
            LogInfo(msg='[patrol_system] Starting USB voice card setup '
                        '(early, chime deferred to cat_voice)'),
            ExecuteProcess(
                cmd=['bash', usb_script],
                name='connect_usb_speaker',
                output='screen',
                additional_env={
                    'CAT_PATROL_SKIP_READY_PROBE': '1',
                    'CAT_PATROL_READY_SOUND': os.path.expanduser(
                        '~/cats/voices/ready-chime.mp3'),
                    'CAT_PATROL_USB_SINK_MATCH': 'USB_PnP_Audio_Device',
                    'CAT_PATROL_USB_VOLUME': '80%',
                },
            ),
        ]))

    # 2 + 3) Localization and Nav2, once odom/TF should be up (~6 s after base).
    entities.append(TimerAction(period=6.0, actions=[
        LogInfo(msg='[patrol_system] Starting localization (AMCL@home) + Nav2'),
        _include('cat_patrol_robot', 'localization.launch.py', launch_arguments=[
            ('params_file', os.path.join(pkg_cat, 'config', 'amcl_params.yaml')),
        ]),
        _include('cat_patrol_robot', 'navigation.launch.py', launch_arguments=[
            ('params_file', os.path.join(pkg_cat, 'config', 'nav2_params.yaml')),
        ]),
    ]))

    # 4 + 5) Brain + detector/voice. When use_voice, USB was started early so
    # cat_voice.launch gets start_usb:=false (avoid racing two setup scripts).
    later = [
        LogInfo(msg='[patrol_system] Starting patrol_manager (waypoint brain)'
                    + (' [require_audio_ready]' if use_voice else '')),
        _include('patrol_manager', 'patrol_manager.launch.py', launch_arguments=[
            ('params_file', os.path.join(pkg_pm, 'config', 'patrol_manager_params.yaml')),
            ('require_audio_ready', 'true' if use_voice else 'false'),
        ]),
        _include('cat_patrol_robot', 'cat_voice.launch.py', launch_arguments=[
            ('start_detector', 'true' if use_detector else 'false'),
            ('start_voice', 'true' if use_voice else 'false'),
            # USB already started early when use_voice; do not run connect twice.
            ('start_usb', 'false'),
            ('start_bt', 'false'),
        ]),
    ]
    if use_voice and not use_detector:
        later.append(LogInfo(
            msg='[patrol_system] WARNING: use_voice=true but use_detector=false — '
                'voice_node will receive no detections and stay silent.'))
    if not use_detector:
        later.append(LogInfo(
            msg='[patrol_system] use_detector=false — cat-recognized email disabled '
                '(no /cat_detector_cpp/detections). Waypoint/loop emails still send.'))
    if use_voice:
        later.append(LogInfo(
            msg='[patrol_system] Audio gate ON: no drive until ready-chime plays '
                'on USB voice card and /cat_patrol/audio_ready latches '
                '(fail-closed, 90s timeout).'))
    entities.append(TimerAction(period=12.0, actions=later))

    # Optional RViz (pose override / live monitoring).
    if use_rviz:
        rviz_cfg = os.path.join(pkg_cat, 'config', 'b.rviz')
        entities.append(TimerAction(period=8.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=(['-d', rviz_cfg] if os.path.isfile(rviz_cfg) else []),
        )]))

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='x3', choices=['x3', 'r2', 'none']),
        DeclareLaunchArgument('start_lidar', default_value='auto',
                              choices=['true', 'false', 'auto']),
        DeclareLaunchArgument('use_detector', default_value='true', choices=['true', 'false'],
                              description='Start cat_detector_cpp (feeds cat-recognized email + voice).'),
        DeclareLaunchArgument('use_voice', default_value='true', choices=['true', 'false'],
                              description='Start voice_node + USB voice card (audio output).'),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false'],
                              description='Also start RViz (for pose override / monitoring).'),
        OpaqueFunction(function=compose),
    ])
