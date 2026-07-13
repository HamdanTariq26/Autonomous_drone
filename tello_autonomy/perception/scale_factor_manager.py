"""
perception/scale_factor_manager.py

ROS2 node that decides WHEN to (re)compute a scale factor for each
active map_id, and dispatches that work to a separate OS PROCESS to do
it - see perception/depth_inference_worker.py for why a process rather
than a thread (short version: threads share their parent's OS
scheduling priority, so a CPU-bound inference thread can still starve
flight control even on its own thread; a process can be os.nice()'d
lower than everything else, a thread cannot).

CHANGED (this revision): recomputation used to be dispatched onto a
threading.Thread per attempt. That thread still ran inference on CPU
inside THIS process, competing for the same OS-scheduled CPU time as
manual_control's flight-control loop and the ROS2 executor thread -
which is what caused the observed control stutter/lag right after
takeoff, correlated with recompute timing. This revision replaces that
per-attempt thread with ONE long-lived worker PROCESS (started once in
__init__, not per-request - process startup via "spawn" plus model
loading is too slow to redo every 15s) that the model lives inside for
the whole node's lifetime. Recompute requests go out over a
multiprocessing.Queue; results come back over a second one, drained by
a lightweight, non-blocking timer on the ROS2 executor thread.

perception.scale_factor's public interface is UNCHANGED - it's now
called from inside the worker process (see depth_inference_worker.py)
instead of a thread in this process, but its function signatures and
behavior are untouched.

Map_id lifecycle tracking (new map_id detection via keyframe_points
arrival, staleness pruning via TOPIC_MAP_TOPOLOGY_CHANGED +
MAP_DATA_STALE_AFTER_SEC) is UNCHANGED from the previous revision - see
that logic below, only the dispatch mechanism changed.
"""

import queue as queue_module
import time

import multiprocessing
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from config import constants
from middleware.topic_manager import TopicManager
from perception.depth_inference_worker import run_worker


def _parse_map_id_from_frame_id(frame_id):
    """Same encoding used by perception.live_scaler - see that module for details."""
    if not frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
        return None
    suffix = frame_id[len(constants.SLAM_MAP_FRAME_ID_PREFIX):]
    try:
        return int(suffix)
    except ValueError:
        return None


# Explicit "spawn" context - deliberately NOT the platform default
# ("fork" on Linux). Forking a process that has already imported
# torch/numpy/OpenMP is a known source of hangs/deadlocks in PyTorch's
# multiprocessing guidance; "spawn" starts a genuinely fresh
# interpreter in the child instead, which costs a bit of startup time
# but avoids that whole class of problem. See depth_inference_worker.py.
_MP_CONTEXT = multiprocessing.get_context("spawn")


class ScaleFactorManager(Node):
    """
        Caches the latest keyframe_points snapshot per map_id (from
        TOPIC_KEYFRAME_POINTS) and runs a per-map_id periodic recompute
        timer, dispatching the actual work to a long-lived worker
        process. Current best scale factor per map_id is available via
        self.current_scale_factors (a plain dict, map_id -> float), for
        any other node/layer (e.g. perception.live_scaler) to read.
    """

    def __init__(self, node_name="scale_factor_manager"):
        super().__init__(node_name)

        self._topics = TopicManager(self)
        self._topics.get_subscription(
            constants.TOPIC_KEYFRAME_POINTS, PointCloud2, self._on_keyframe_points
        )
        self._topics.get_subscription(
            constants.TOPIC_MAP_TOPOLOGY_CHANGED, Bool, self._on_topology_changed
        )

        # map_id -> {"timer": rclpy Timer}
        self._map_state = {}

        # map_id -> list of row dicts (latest received snapshot for
        # that map_id) - same schema as before (keyframe_id, timestamp,
        # pixel_u, pixel_v, depth_camera_frame), map_id column added
        # back in only when building a request for the worker.
        self._map_points_cache = {}

        # map_id -> time.time() this node last received a
        # keyframe_points message for it. Used for staleness pruning.
        self._last_seen = {}

        # map_ids with a request currently queued or being processed by
        # the worker - prevents piling up redundant duplicate requests
        # for the same map_id while one is still in flight.
        self._in_flight_map_ids = set()

        # map_id -> most recently computed scale factor (float).
        # Instance attribute (not class-level) - each ScaleFactorManager
        # instance has its own independent dict.
        self.current_scale_factors = {}

        # ---- long-lived worker process: started once here, not once
        # per recompute. The model loads ONCE inside the worker process
        # and stays resident for this node's entire lifetime. ----
        self._request_queue = _MP_CONTEXT.Queue()
        self._result_queue = _MP_CONTEXT.Queue()
        self._worker_process = _MP_CONTEXT.Process(
            target=run_worker,
            args=(self._request_queue, self._result_queue),
            daemon=True,
        )
        self._worker_process.start()

        # Drains _result_queue on the ROS2 executor thread. Safe to run
        # frequently - get_nowait() never blocks, so this can never
        # cause the stall this whole redesign exists to eliminate.
        self._result_poll_timer = self.create_timer(
            constants.DEPTH_WORKER_RESULT_POLL_INTERVAL_SEC, self._poll_results
        )

        self.get_logger().info(
            f"ScaleFactorManager: subscribed to {constants.TOPIC_KEYFRAME_POINTS} "
            f"and {constants.TOPIC_MAP_TOPOLOGY_CHANGED}. Depth inference worker "
            f"process started (pid={self._worker_process.pid})."
        )

    # ****************************************************************************************
    def _on_keyframe_points(self, msg: PointCloud2):
        """
            One message = one full snapshot of a single map_id's
            current keyframe points, published periodically by the C++
            node's PublishLiveMapData(). Replaces this map_id's cached
            rows entirely (not appended).
        """
        map_id = _parse_map_id_from_frame_id(msg.header.frame_id)
        if map_id is None:
            self.get_logger().warn(
                f"keyframe_points message with unparseable frame_id '{msg.header.frame_id}' - ignoring."
            )
            return

        rows = []
        for kf_id, ts, u, v, depth in point_cloud2.read_points(
            msg, field_names=("keyframe_id", "keyframe_timestamp", "pixel_u", "pixel_v", "depth"),
            skip_nans=False,
        ):
            rows.append({
                "keyframe_id": int(kf_id),
                "timestamp": float(ts),
                "pixel_u": float(u),
                "pixel_v": float(v),
                "depth_camera_frame": float(depth),
            })

        is_new_map_id = map_id not in self._map_points_cache
        self._map_points_cache[map_id] = rows
        self._last_seen[map_id] = time.time()

        if is_new_map_id:
            self._start_tracking(map_id)

    # ****************************************************************************************
    def _on_topology_changed(self, msg):
        """
            Fired whenever the C++ node's set of active map_ids
            changes - exactly the right moment to check for map_ids
            whose keyframe_points messages have gone stale (merged
            away, C++ side stopped publishing for them).
        """
        self._prune_stale_map_ids()

    # ****************************************************************************************
    def _prune_stale_map_ids(self):
        now = time.time()
        stale_map_ids = [
            map_id for map_id, last_seen in self._last_seen.items()
            if (now - last_seen) > constants.MAP_DATA_STALE_AFTER_SEC
        ]
        for map_id in stale_map_ids:
            self._stop_tracking(map_id)

    # ****************************************************************************************
    def _start_tracking(self, map_id):
        """
            Begins periodic recompute tracking for a newly-seen map_id:
            starts a repeating timer at constants.
            PERIODIC_RECALIBRATION_SECONDS, and attempts an immediate
            first dispatch rather than waiting a full interval for the
            first result.
        """
        self.get_logger().info(f"New map_id {map_id} detected - starting scale-factor tracking.")
        timer = self.create_timer(
            constants.PERIODIC_RECALIBRATION_SECONDS,
            lambda: self._trigger_recompute(map_id),
        )
        self._map_state[map_id] = {"timer": timer}
        self._trigger_recompute(map_id)

    # ****************************************************************************************
    def _stop_tracking(self, map_id):
        """
            Drops all per-map_id state for a map_id that's gone stale
            (merged away) - cancels its timer and discards its cached
            scale factor, cached points, last-seen timestamp, and
            in-flight tracking.
        """
        state = self._map_state.pop(map_id, None)
        if state is not None and state["timer"] is not None:
            state["timer"].cancel()
        self.current_scale_factors.pop(map_id, None)
        self._map_points_cache.pop(map_id, None)
        self._last_seen.pop(map_id, None)
        self._in_flight_map_ids.discard(map_id)
        self.get_logger().info(f"map_id {map_id} no longer active - dropped its scale-factor state.")

    # ****************************************************************************************
    def _trigger_recompute(self, map_id):
        """
            Called from a ROS2 timer callback (the executor thread).
            Enqueues a request for the WORKER PROCESS and returns
            immediately - never runs inference itself, and never
            blocks. Skips enqueueing if a request for this map_id is
            already in flight (queued or currently being processed),
            rather than piling up redundant work the worker can't keep
            up with.
        """
        if map_id in self._in_flight_map_ids:
            self.get_logger().debug(
                f"map_id {map_id}: previous recompute still in flight - skipping this tick."
            )
            return

        rows = self._map_points_cache.get(map_id)
        if not rows:
            return

        num_keyframes = len({row["keyframe_id"] for row in rows})
        if num_keyframes < constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR:
            self.get_logger().info(
                f"map_id {map_id}: only {num_keyframes} keyframes cached so far "
                f"(need {constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR}) - skipping this tick."
            )
            return

        self._in_flight_map_ids.add(map_id)
        self._request_queue.put({"map_id": map_id, "rows": rows})

    # ****************************************************************************************
    def _poll_results(self):
        """
            Non-blocking drain of every result currently sitting in
            _result_queue. Runs on the ROS2 executor thread but never
            blocks - queue_module.Empty just means "nothing new yet,"
            not an error.
        """
        while True:
            try:
                result = self._result_queue.get_nowait()
            except queue_module.Empty:
                break

            map_id = result["map_id"]
            self._in_flight_map_ids.discard(map_id)

            if map_id not in self._map_state:
                # map_id went stale while this result was in flight -
                # discard rather than resurrect dropped state.
                continue

            if result.get("error"):
                self.get_logger().error(f"map_id {map_id}: worker raised: {result['error']}")
                continue

            if result["scale_factor"] is None:
                self.get_logger().warn(f"map_id {map_id}: no usable point ratios this attempt.")
                continue

            self.current_scale_factors[map_id] = result["scale_factor"]
            self.get_logger().info(
                f"map_id {map_id}: scale_factor={result['scale_factor']:.6f} "
                f"(from {result['num_points_used']} points across "
                f"{result['num_keyframes_used']} keyframes, "
                f"{result['num_keyframes_skipped']} skipped)"
            )

    # ****************************************************************************************
    def destroy_node(self):
        """
            Cancels every per-map_id timer plus the result-poll timer,
            signals the worker process to exit via the sentinel value,
            joins it (falling back to terminate() if it doesn't exit
            cleanly), and cleans up the subscription before shutdown.
        """
        for map_id in list(self._map_state.keys()):
            self._stop_tracking(map_id)
        if self._result_poll_timer is not None:
            self._result_poll_timer.cancel()

        self._request_queue.put(None)  # sentinel - see depth_inference_worker.run_worker
        self._worker_process.join(timeout=5.0)
        if self._worker_process.is_alive():
            self.get_logger().warn("Depth inference worker didn't exit cleanly - terminating.")
            self._worker_process.terminate()

        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
