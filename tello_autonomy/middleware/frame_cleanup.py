"""
middleware/frame_cleanup.py

Standalone ROS2 node, separate from RosBridge. Its only job: watch
constants.TOPIC_KEYFRAME_TIMESTAMPS (published live by the C++ SLAM
node, ~1Hz) and delete saved frame images from constants.SAVE_FRAMES_DIR
once we're confident they did NOT become a keyframe.

Ported from the old flat-file frame_cleanup_node.py - logic is
unchanged, only the wiring changed: topic name, save-frames dir, and
the three tuning constants (match tolerance / grace period / cleanup
interval) now come from config.constants instead of constructor
defaults, and the subscription goes through middleware.topic_manager
like everything else in this layer.

Deletion logic, and why the grace period matters:
ORB-SLAM3 decides whether a frame becomes a keyframe on a separate
thread (LocalMapping), with some delay after the frame is first
processed. If we delete a frame the instant it's absent from the
current keyframe list, we might delete one that's still mid-pipeline
and about to be promoted to a keyframe a moment later. So a frame is
only ever considered "safe to delete" once BOTH are true:
  1. Its timestamp doesn't match any timestamp in the most recent
     keyframe list from the C++ node (within KEYFRAME_MATCH_TOLERANCE_SEC)
  2. It's older than DELETION_GRACE_PERIOD_SEC (giving LocalMapping
     time to catch up before we give up on it)

Runs as its own ROS2 node, on its own executor/thread - see the
two-executor pattern used in the old main.py (one SingleThreadedExecutor
per node) to avoid driving two nodes' callbacks from one shared
executor across threads.
"""

import glob
import os
import re
import time

from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

from config import constants
from middleware.topic_manager import TopicManager


class FrameCleanupNode(Node):
    """
        Subscribes to the live keyframe-timestamp list and periodically
        deletes saved frame images that are old enough, and were never
        matched to a keyframe timestamp, to be considered safe to
        discard. Never deletes anything before the first keyframe list
        has been received - an empty/unset list must never be treated
        as "nothing is a keyframe, delete everything".
    """

    def __init__(self, node_name="frame_cleanup_node",
                 save_frames_dir=None,
                 keyframe_match_tolerance=None,
                 deletion_grace_period=None,
                 cleanup_interval=None):
        """
            All four tuning parameters default to config.constants
            values - pass an override only if a specific run genuinely
            needs different behavior (e.g. a test harness).
        """
        super().__init__(node_name)

        self.save_frames_dir = save_frames_dir or constants.SAVE_FRAMES_DIR
        self.keyframe_match_tolerance = (
            keyframe_match_tolerance
            if keyframe_match_tolerance is not None
            else constants.KEYFRAME_MATCH_TOLERANCE_SEC
        )
        self.deletion_grace_period = (
            deletion_grace_period
            if deletion_grace_period is not None
            else constants.DELETION_GRACE_PERIOD_SEC
        )
        self.cleanup_interval = (
            cleanup_interval
            if cleanup_interval is not None
            else constants.CLEANUP_INTERVAL_SEC
        )

        self.current_keyframe_timestamps = []     # latest list received from C++ node
        self.have_received_keyframe_list = False  # gate: don't delete anything until we've heard from C++ at least once

        # filenames look like: frame_1783539824.595438_000123.png
        self._frame_filename_re = re.compile(r"frame_(\d+\.\d+)_(\d+)\.png$")

        self._topics = TopicManager(self)
        self._topics.get_subscription(
            constants.TOPIC_KEYFRAME_TIMESTAMPS,
            Float64MultiArray,
            self._on_keyframe_timestamps,
        )

        self.cleanup_timer = self.create_timer(self.cleanup_interval, self._cleanup_tick)

        self.total_deleted = 0

        self.get_logger().info(
            f"FrameCleanupNode: save_frames_dir={self.save_frames_dir}, "
            f"keyframe_match_tolerance={self.keyframe_match_tolerance}s, "
            f"deletion_grace_period={self.deletion_grace_period}s, "
            f"cleanup_interval={self.cleanup_interval}s. "
            f"Waiting for first keyframe timestamp list before deleting anything..."
        )

    # ****************************************************************************************
    def _on_keyframe_timestamps(self, msg):
        self.current_keyframe_timestamps = list(msg.data)
        if not self.have_received_keyframe_list:
            self.have_received_keyframe_list = True
            self.get_logger().info(
                f"First keyframe list received ({len(self.current_keyframe_timestamps)} "
                f"keyframes) - cleanup is now active."
            )

    # ****************************************************************************************
    def _is_keyframe_timestamp(self, frame_ts):
        """True if frame_ts is within tolerance of any current keyframe timestamp."""
        for kf_ts in self.current_keyframe_timestamps:
            if abs(frame_ts - kf_ts) <= self.keyframe_match_tolerance:
                return True
        return False

    # ****************************************************************************************
    def _cleanup_tick(self):
        # Watchdog: never delete anything before we've heard from the
        # C++ node at least once.
        if not self.have_received_keyframe_list:
            return

        now = time.time()
        pattern = os.path.join(self.save_frames_dir, "frame_*.png")
        deleted_this_pass = 0

        for filepath in glob.glob(pattern):
            filename = os.path.basename(filepath)
            match = self._frame_filename_re.match(filename)
            if not match:
                continue  # unexpected filename, leave it alone rather than guess

            frame_ts = float(match.group(1))
            age = now - frame_ts

            if age < self.deletion_grace_period:
                continue  # too young, LocalMapping might still promote it - leave it

            if self._is_keyframe_timestamp(frame_ts):
                continue  # this one IS a keyframe - keep it

            try:
                os.remove(filepath)
                deleted_this_pass += 1
            except OSError as e:
                self.get_logger().warn(f"Failed to delete {filepath}: {e}")

        if deleted_this_pass > 0:
            self.total_deleted += deleted_this_pass
            self.get_logger().info(
                f"Cleanup pass: deleted {deleted_this_pass} non-keyframe images "
                f"(total deleted so far: {self.total_deleted})"
            )

    # ****************************************************************************************
    def destroy_node(self):
        """Cancels the cleanup timer and cleans up the subscription before shutdown."""
        if self.cleanup_timer is not None:
            self.cleanup_timer.cancel()
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************