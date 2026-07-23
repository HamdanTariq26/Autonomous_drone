#!/usr/bin/env python3
"""
scripts/main.py

THE entry point that wires every built layer together and runs a full
manual-flight-with-live-SLAM-and-scale-factor session:

  drone_interface (no ROS2):
    TelloDriver -> FrameReceiver, TelemetryMonitor, CommandHandler -> ManualControl

  middleware (ROS2):
    RosBridge          (publishes frames+handshake to the C++ SLAM node)
    FrameCleanupNode   (deletes non-keyframe saved frame images)

  perception (ROS2, depends on middleware's topics):
    ScaleFactorManager (computes per-map_id scale factors)
    LiveScaler         (multiplies live pose/points by the current scale factor)

The C++ SLAM node (ros2_orb_slam3 / mono_node_cpp) is a SEPARATE
process, started independently before or after this script - it is not
launched from here. Run it with:
    ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

Each ROS2 node gets its OWN SingleThreadedExecutor on its own thread -
this is deliberate (see architecture doc / earlier debugging): driving
multiple nodes' callbacks from a single shared executor across threads
risks "ValueError: generator already executing".

drone_interface's ManualControl loop is NOT tied to any ROS2 executor -
it runs independently (drone_interface never depends on middleware),
so manual flight control works immediately after connecting, regardless
of whether the SLAM handshake has completed yet.

Usage:
    cd ~/Autonomous_Drone/ros2_test
    source /opt/ros/humble/setup.bash
    source install/setup.bash    # once this is a proper colcon package
    python3 tello_autonomy/scripts/main.py
"""

import os
import sys
import threading

_TELLO_AUTONOMY_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TELLO_AUTONOMY_ROOT not in sys.path:
    sys.path.insert(0, _TELLO_AUTONOMY_ROOT)

import rclpy
from rclpy.executors import SingleThreadedExecutor

from config import constants
from drone_interface.tello_driver import TelloDriver
from drone_interface.frame_receiver import FrameReceiver
from drone_interface.telemetry import TelemetryMonitor
from drone_interface.command_handler import CommandHandler
from drone_interface.manual_control import ManualControl
from middleware.ros_bridge import RosBridge
from middleware.frame_cleanup import FrameCleanupNode
from perception.scale_factor_manager import ScaleFactorManager
from perception.live_scaler import LiveScaler
from goals.mission_controller import MissionControllerNode


class NodeRunner:
    """
        Tiny helper: owns one ROS2 node + its own SingleThreadedExecutor
        + its own spin thread. Exists so main() doesn't repeat this
        three/four times with copy-pasted thread/executor boilerplate.
    """
    def __init__(self, node):
        self.node = node
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)

    def start(self):
        self._thread.start()

    def shutdown(self, join_timeout=2.0):
        self._executor.shutdown()
        self._thread.join(timeout=join_timeout)
        self.node.destroy_node()


def main():
    # ---- drone_interface layer: no ROS2 involved yet ----
    driver = TelloDriver(resolution="", fps="", bitrate=constants.TELLO_VIDEO_BITRATE)
    driver.connect()

    frame_receiver = FrameReceiver(driver)
    telemetry = TelemetryMonitor(driver, enabled=True)  # default fields: battery, height, flight_time
    command_handler = CommandHandler(driver)
    manual_control = ManualControl(command_handler, frame_receiver, manual_speed=constants.DEFAULT_MANUAL_SPEED_CMS)

    # ---- ROS2 layer: middleware + perception + goals ----
    rclpy.init()

    ros_bridge = RosBridge(frame_receiver, node_name="tello_ros_bridge")
    frame_cleanup = FrameCleanupNode(node_name="frame_cleanup_node")
    scale_factor_manager = ScaleFactorManager(node_name="scale_factor_manager")
    live_scaler = LiveScaler(scale_factor_manager, node_name="live_scaler")
    mission_controller = MissionControllerNode(command_handler, node_name="mission_controller")

    runners = [
        NodeRunner(ros_bridge),
        NodeRunner(frame_cleanup),
        NodeRunner(scale_factor_manager),
        NodeRunner(live_scaler),
        NodeRunner(mission_controller),
    ]
    for runner in runners:
        runner.start()

    # ---- Wire manual_control's quit -> full shutdown, and start it.
    # ManualControl runs independent of ROS2 entirely - it doesn't wait
    # for the SLAM handshake to complete before allowing takeoff. ----
    shutdown_event = threading.Event()

    def on_quit():
        shutdown_event.set()

    manual_control.on_quit_requested = on_quit
    manual_control.on_manual_override = mission_controller.cancel_mission
    
    # Return to home hover point (0, 0, 1.0m height) when 'h' is pressed.
    # We don't use z=0 because the floor is mapped as occupied space at z=0!
    manual_control.on_rth_requested = lambda: mission_controller.start_mission_to_goal(0.0, -1.0, 0.0)
    
    # Notify user when scale factor is ready
    scale_factor_manager.on_scale_computing = lambda map_id: \
        manual_control.set_scale_status("SCALE: Computing...", (0, 200, 255))   # yellow
    scale_factor_manager.on_scale_ready = lambda map_id, scale: \
        manual_control.set_scale_status(f"SCALE: Ready  {scale:.2f}", (0, 255, 0))  # green
    scale_factor_manager.on_scale_lost = lambda: \
        manual_control.set_scale_status("SCALE: Lost - Tracking Failure", (0, 0, 255))    # red
    
    command_handler.start_keepalive()
    telemetry.enable()
    print("Pipeline running. Fly manually (t=takeoff, q=land+quit). "
          "SLAM handshake, scale-factor computation, and live scaling "
          "run automatically once the C++ SLAM node ACKs.")

    try:
        # Run in main thread! This handles OpenCV UI and blocks until shutdown.
        manual_control.start(window_name="Tello SLAM View", blocking=True)
    except KeyboardInterrupt:
        print("\nCtrl+C received - shutting down.")
        shutdown_event.set()
    finally:
        print("Shutting down drone_interface...")
        manual_control.stop()
        command_handler.stop_keepalive()
        driver.disconnect()

        print("Shutting down ROS2 nodes...")
        for runner in runners:
            runner.shutdown()

        rclpy.shutdown()
        print("Clean shutdown complete.")


if __name__ == "__main__":
    main()
