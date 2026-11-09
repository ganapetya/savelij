# =============================================================================
# cat_voice.launch.py — cat detection + per-cat voice playback (toggleable)
# =============================================================================
#
# Standalone launch file, run ALONGSIDE the patrol stack -- it does not touch
# the patrol brains directly (it only publishes /cat_detector_cpp/detections,
# which the brains subscribe to). Independently-toggleable pieces:
#
#   start_detector (default true)  cat_detector_cpp -- YOLO + per-cat (white/
#                                  brown) classifier. Publishes identity on
#                                  /cat_detector_cpp/detections. Required for
#                                  both voice playback AND the cat-recognized
#                                  email in the patrol brain.
#   start_voice    (default true)  voice_node -- subscribes to that topic and
#                                  plays ~/cats/voices/{jente,arik}-*.mp3 on
#                                  recognition. Also runs the ready-chime
#                                  probe and latches /cat_patrol/audio_ready.
#   start_usb      (default true)  connect_usb_speaker.sh -- sets PulseAudio
#                                  default sink to the USB voice card (JMTek
#                                  0c76:1203). Preferred path (wired speakers).
#   start_bt       (default false) connect_bt_speaker.sh -- legacy Bluetooth
#                                  A2DP path. Leave false unless you need BT.
#
# When start_voice is true, the sink-setup script skips its own chime
# (CAT_PATROL_SKIP_READY_PROBE) so only voice_node plays the ready file once.
#
# USAGE:
#   ros2 launch cat_patrol_robot cat_voice.launch.py
#   ros2 launch cat_patrol_robot cat_voice.launch.py start_usb:=false start_bt:=true
# =============================================================================
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_detector = get_package_share_directory('cat_detector_cpp')
    lib_dir = os.path.join(get_package_prefix('cat_patrol_robot'), 'lib', 'cat_patrol_robot')

    usb_connect_script = os.path.join(lib_dir, 'connect_usb_speaker.sh')
    bt_connect_script = os.path.join(lib_dir, 'connect_bt_speaker.sh')

    start_usb_arg = DeclareLaunchArgument(
        'start_usb', default_value='true', choices=['true', 'false'],
        description='Set PulseAudio default sink to the USB voice card (0c76:1203).')
    start_bt_arg = DeclareLaunchArgument(
        'start_bt', default_value='false', choices=['true', 'false'],
        description='Legacy: connect BT speaker and set bluez as default sink.')
    start_detector_arg = DeclareLaunchArgument(
        'start_detector', default_value='true', choices=['true', 'false'],
        description='Start cat_detector_cpp (YOLO + white/brown classifier).')
    start_voice_arg = DeclareLaunchArgument(
        'start_voice', default_value='true', choices=['true', 'false'],
        description='Start voice_node (plays per-cat mp3 on recognition + ready probe).')

    # When voice_node will run the ready-chime probe, skip the duplicate play
    # inside the connect scripts (still set the default sink).
    audio_env = {
        'CAT_PATROL_SKIP_READY_PROBE': '1',
        'CAT_PATROL_READY_SOUND': os.path.expanduser('~/cats/voices/ready-chime.mp3'),
        'CAT_PATROL_USB_SINK_MATCH': 'USB_PnP_Audio_Device',
        'CAT_PATROL_USB_VOLUME': '80%',
    }

    usb_connect = ExecuteProcess(
        cmd=['bash', usb_connect_script],
        name='connect_usb_speaker',
        output='screen',
        additional_env=audio_env,
        condition=IfCondition(LaunchConfiguration('start_usb')),
    )

    bt_connect = ExecuteProcess(
        cmd=['bash', bt_connect_script],
        name='connect_bt_speaker',
        output='screen',
        additional_env=audio_env,
        condition=IfCondition(LaunchConfiguration('start_bt')),
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_detector, 'launch', 'cat_detector_cpp.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_detector')),
    )

    voice_node = Node(
        package='cat_patrol_robot',
        executable='voice_node',
        name='cat_voice',
        output='screen',
        parameters=[os.path.join(pkg_cat, 'config', 'cat_voice_params.yaml')],
        condition=IfCondition(LaunchConfiguration('start_voice')),
    )

    return LaunchDescription([
        start_usb_arg,
        start_bt_arg,
        start_detector_arg,
        start_voice_arg,
        usb_connect,
        bt_connect,
        detector,
        voice_node,
    ])
