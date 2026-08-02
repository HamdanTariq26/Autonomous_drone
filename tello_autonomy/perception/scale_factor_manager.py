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
from std_msgs.msg import Bool, Float64, Int32, String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from config import constants
from middleware.topic_manager import TopicManager
from perception.depth_inference_worker import run_worker
from perception.pose_transform import transform_camera_points_to_world


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

    def __init__(self, node_name="scale_factor_manager", tof_estimator=None):
        super().__init__(node_name)

        # Optional reference to a ToFScaleEstimator instance (same process).
        # None means ToF source is unavailable (e.g. simulation, or before
        # the node starts).  ScaleFactorManager never creates this itself -
        # scripts/main.py wires it in.
        self._tof_estimator = tof_estimator

        # Runtime scale-source mode, initialised from constants but mutable
        # at runtime via cycle_scale_source_mode() (bound to 'm' key).
        self.scale_source_mode = constants.SCALE_SOURCE_MODE

        # Raw Depth-Anything-V2 results, stored separately from
        # current_scale_factors so the arbiter can always see both
        # sources independently regardless of the active mode.
        self._depth_scale_factors = {}

        self._topics = TopicManager(self)
        self._scale_factor_pub = self.create_publisher(
            Float64, "/tello_autonomy/scale_factor", 10
        )
        self._debug_pub = self.create_publisher(
            String, "/tello_autonomy/scale_source_debug", 10
        )
        self._dense_points_pub = self.create_publisher(
            PointCloud2, constants.TOPIC_DENSE_POINTS_METRIC, 10
        )
        self._dense_pose_pub = self.create_publisher(
            PoseStamped, "/tello_autonomy/dense_pose_metric", 10
        )
        self._topics.get_subscription(
            constants.TOPIC_KEYFRAME_POINTS, PointCloud2, self._on_keyframe_points
        )
        self._topics.get_subscription(
            constants.TOPIC_MAP_TOPOLOGY_CHANGED, Int32, self._on_topology_changed
        )
        self._topics.get_subscription(
            constants.TOPIC_TRAJECTORY, Path, self._on_trajectory
        )

        # map_id -> list of PoseStamped from TOPIC_TRAJECTORY
        self._trajectory_cache = {}

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

        # pipeline_audit.md Finding #3: map_id -> {"candidate": float,
        # "confirmations": int} for a scale-factor jump that exceeded
        # constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD relative to the
        # currently active value and has not yet been confirmed enough
        # times to be adopted. See _resolve_active_scale() /
        # _maybe_adopt_scale() for the actual quarantine logic. Absence
        # of a map_id here means "no quarantined candidate right now" -
        # either nothing large enough to quarantine has been seen, or a
        # previous quarantine was already resolved (adopted or
        # superseded).
        self._quarantine = {}
        
        # Callbacks
        self.on_scale_ready = None
        self.on_scale_computing = None
        self.on_scale_lost = None

        self._last_known_map_count = 0

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

        # Re-resolves the active scale factor for every tracked map_id at
        # the same cadence as the depth-result poll.  This is what lets
        # ToF's much faster updates actually reach current_scale_factors
        # in "auto"/"tof" mode without waiting for the next Depth-Anything
        # recompute cycle (which can take 30-60s).
        self._resolve_timer = self.create_timer(
            constants.DEPTH_WORKER_RESULT_POLL_INTERVAL_SEC, self._resolve_all_active_scales
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
        old_count = self._last_known_map_count
        self._last_known_map_count = msg.data
        self._prune_stale_map_ids(old_count)

    # ****************************************************************************************
    def _on_trajectory(self, msg: Path):
        """
            Caches keyframe poses per map_id from TOPIC_TRAJECTORY.
            Used to position back-projected dense point clouds in the world frame.
        """
        map_id = _parse_map_id_from_frame_id(msg.header.frame_id)
        if map_id is not None:
            self._trajectory_cache[map_id] = msg.poses

    # ****************************************************************************************

    # ****************************************************************************************
    def _prune_stale_map_ids(self, prev_c_map_count=None):
        now = time.time()
        stale_map_ids = [
            map_id for map_id, last_seen in self._last_seen.items()
            if (now - last_seen) > constants.MAP_DATA_STALE_AFTER_SEC
        ]
        for map_id in stale_map_ids:
            self._stop_tracking(map_id)

        # Fire on_scale_lost ONCE after all pruning, using count direction
        # to distinguish loss from merge:
        #   count went DOWN  -> merge (maps consolidated) -> no alarm
        #   count same or UP -> loss (new map from restart) -> fire red
        if stale_map_ids and self.on_scale_lost is not None and prev_c_map_count is not None:
            if self._last_known_map_count >= prev_c_map_count:
                self.on_scale_lost()

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
        if self.on_scale_computing is not None:
            self.on_scale_computing(map_id)
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
        self._depth_scale_factors.pop(map_id, None)
        self._map_points_cache.pop(map_id, None)
        self._last_seen.pop(map_id, None)
        self._in_flight_map_ids.discard(map_id)
        self._quarantine.pop(map_id, None)
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

            self._depth_scale_factors[map_id] = result["scale_factor"]
            self._resolve_active_scale(map_id)

            if result.get("dense_points_cam") is not None:
                self._publish_dense_depth_points(result)

            # on_scale_ready now fires from _resolve_active_scale, so it
            # always reflects whichever source actually won (ToF or Depth-AI).
            self.get_logger().info(
                f"map_id {map_id}: scale_factor={result['scale_factor']:.6f} "
                f"(from {result['num_points_used']} points across "
                f"{result['num_keyframes_used']} keyframes, "
                f"{result['num_keyframes_skipped']} skipped)"
            )

    # ****************************************************************************************
    def _publish_dense_depth_points(self, result: dict):
        """
            Transforms camera-frame dense points from Depth-Anything backprojection
            into the world frame using the keyframe's pose and current metric scale factor,
            then publishes on TOPIC_DENSE_POINTS_METRIC.
        """
        points_cam = result.get("dense_points_cam")
        map_id = result.get("map_id")

        if points_cam is None or len(points_cam) == 0 or map_id is None:
            return

        active_scale = self.current_scale_factors.get(map_id)
        if not active_scale or active_scale <= 0:
            return

        poses = self._trajectory_cache.get(map_id, [])
        if not poses:
            return

        matched_pose = poses[-1] if poses else None
        if matched_pose is None:
            return

        raw_pos = matched_pose.pose.position
        orientation = matched_pose.pose.orientation
        translation_m = (raw_pos.x * active_scale, raw_pos.y * active_scale, raw_pos.z * active_scale)

        points_world = transform_camera_points_to_world(
            points_cam=points_cam,
            orientation=orientation,
            translation_m=translation_m,
        )

        frame_id_str = f"{constants.SLAM_MAP_FRAME_ID_PREFIX}{map_id}"
        now_stamp = self.get_clock().now().to_msg()

        dense_pose_msg = PoseStamped()
        dense_pose_msg.header.stamp = now_stamp
        dense_pose_msg.header.frame_id = frame_id_str
        dense_pose_msg.pose.position.x = translation_m[0]
        dense_pose_msg.pose.position.y = translation_m[1]
        dense_pose_msg.pose.position.z = translation_m[2]
        dense_pose_msg.pose.orientation = orientation
        self._dense_pose_pub.publish(dense_pose_msg)

        header = dense_pose_msg.header
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points_world.tolist())
        self._dense_points_pub.publish(cloud_msg)
        self.get_logger().info(
            f"map_id {map_id}: Published {len(points_world)} dense metric points to {constants.TOPIC_DENSE_POINTS_METRIC}."
        )

    # ****************************************************************************************
    def _resolve_active_scale(self, map_id: int):
        """
            Picks the active scale factor for map_id from whichever
            source(s) are available, according to self.scale_source_mode,
            then routes it through _maybe_adopt_scale() (pipeline_audit.md
            Finding #3) before writing to self.current_scale_factors — the
            dict perception.live_scaler.LiveScaler reads. Nothing
            downstream needs to know which source won or whether a
            quarantine delay was involved.

            Also publishes to /tello_autonomy/scale_factor and fires
            on_scale_ready so the HUD overlay stays in sync regardless
            of whether a depth or ToF result triggered the call - but
            only once a value is actually adopted, not on every call
            that merely proposes one.
        """
        depth_scale = self._depth_scale_factors.get(map_id)
        tof_scale, tof_reliable = None, False
        if self._tof_estimator is not None:
            tof_scale, tof_reliable = self._tof_estimator.get_scale_estimate(map_id)

        mode = self.scale_source_mode
        if mode == "tof":
            # Prefer ToF; fall back to last depth result if ToF not ready yet.
            chosen = tof_scale if tof_scale is not None else depth_scale
        elif mode == "depth":
            chosen = depth_scale
        else:  # "auto"
            # Prefer ToF when it's reliable (low std-dev, enough samples);
            # fall back to Depth-Anything otherwise.
            chosen = (tof_scale if (tof_scale is not None and tof_reliable)
                      else depth_scale)

        if chosen is None:
            return

        adopted_value = self._maybe_adopt_scale(map_id, chosen)
        if adopted_value is None:
            return  # candidate is quarantined - current_scale_factors untouched this call

        self.current_scale_factors[map_id] = adopted_value

        _sf_msg = Float64()
        _sf_msg.data = adopted_value
        self._scale_factor_pub.publish(_sf_msg)

        dbg = String()
        dbg.data = f"map_id={map_id} mode={mode} tof_reliable={tof_reliable}"
        self._debug_pub.publish(dbg)

        if self.on_scale_ready is not None:
            self.on_scale_ready(map_id, adopted_value)

    # ****************************************************************************************
    def _maybe_adopt_scale(self, map_id: int, candidate: float):
        """
            pipeline_audit.md Finding #3: gate between "a source proposed
            this scale factor" and "current_scale_factors[map_id] is
            actually updated to it". occupancy_map_cpp's OcTree already
            contains points inserted under whatever scale was active
            before this call - there's no mechanism reconciling old
            insertions with a corrected scale, so a large, sudden jump
            (calibrate_scale_factor.py's own docstring cites a real
            example: "suggested_scale = current * 1.35") would silently
            smear/duplicate geometry in the live map the instant it took
            effect, with no visible error.

            Returns the value that should be written to
            current_scale_factors[map_id] THIS call, or None if the
            candidate is being held in quarantine and nothing should be
            adopted yet (the caller must leave current_scale_factors
            untouched in that case - the previous value, if any, simply
            stays active).

            IMPORTANT SUBTLETY this implementation has to account for:
            _resolve_active_scale() is called both from _poll_results()
            (only when a genuinely NEW Depth-Anything result arrives,
            every PERIODIC_RECALIBRATION_SECONDS) and from
            _resolve_all_active_scales() (on a fast ~0.5s timer, for
            EVERY tracked map_id, regardless of whether anything
            actually changed - this is what lets ToF's fast updates
            reach current_scale_factors promptly). In "depth" mode
            between two real recomputes, that fast timer re-proposes the
            exact same stale depth_scale value roughly 30-120 times
            before the next actual measurement. If confirmations were
            counted per CALL rather than per genuinely new PROPOSAL, a
            single bad Depth-Anything result would rack up
            QUARANTINE_CONFIRMATIONS_REQUIRED confirmations within about
            a second of wall-clock time - defeating the entire point of
            requiring the jump to be seen again on a SEPARATE
            measurement. So: a confirmation is only counted when
            `candidate` differs (beyond floating-point noise) from the
            candidate this same map_id was quarantined against on the
            previous call - i.e. the source must have actually produced
            a new number, not merely been asked again.

            Logic:
              - No active value for this map_id yet (this is the very
                first scale factor ever computed for it): adopt
                immediately. There's nothing to "jump" from, and
                withholding a map's first-ever scale factor would only
                delay when live_scaler starts publishing metric poses
                for no safety benefit.
              - Candidate within QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD of
                the active value: adopt immediately, same as before this
                fix - small, gradual updates (which is most of them,
                especially from the fast-updating ToF path) are not
                delayed by any of this.
              - Candidate is a sharp jump from the active value: don't
                adopt outright. If the exact same candidate value was
                already being quarantined, count this as a repeat
                observation only if it looks like a genuinely new
                measurement (see subtlety above) - simplified here by
                comparing against the previous call's candidate rather
                than trying to detect "new measurement" at this layer.
                Once confirmations reach QUARANTINE_CONFIRMATIONS_REQUIRED
                distinct-looking proposals, the jump is treated as a
                genuine, sustained correction and IS adopted, with the
                quarantine entry cleared.
        """
        active = self.current_scale_factors.get(map_id)

        if active is None or active == 0.0:
            return candidate  # nothing to compare against - adopt the first value outright

        jump_ratio = abs(candidate - active) / abs(active)
        if jump_ratio <= constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD:
            # Small, gradual change - adopt immediately and clear any
            # stale quarantine entry (the source has evidently settled
            # back down near the active value on its own).
            self._quarantine.pop(map_id, None)
            return candidate

        held = self._quarantine.get(map_id)
        if held is None:
            self._quarantine[map_id] = {
                "candidate": candidate,
                "confirmations": 1,
                "last_proposal": candidate,
            }
            self.get_logger().warn(
                f"map_id {map_id}: scale factor jump {active:.6f} -> {candidate:.6f} "
                f"({jump_ratio * 100:.1f}%) exceeds "
                f"{constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD * 100:.0f}% threshold - "
                f"quarantining (1/{constants.QUARANTINE_CONFIRMATIONS_REQUIRED} confirmations)."
            )
            return None

        held_candidate = held["candidate"]
        agreement_ratio = abs(candidate - held_candidate) / abs(held_candidate) if held_candidate != 0.0 else 1.0
        if agreement_ratio > constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD:
            # The jump target itself moved since last time - still noisy,
            # restart the quarantine against this newest candidate rather
            # than confirming a moving target.
            self._quarantine[map_id] = {
                "candidate": candidate,
                "confirmations": 1,
                "last_proposal": candidate,
            }
            self.get_logger().warn(
                f"map_id {map_id}: quarantined candidate moved ({held_candidate:.6f} -> "
                f"{candidate:.6f}) - restarting confirmation count "
                f"(1/{constants.QUARANTINE_CONFIRMATIONS_REQUIRED})."
            )
            return None

        # Candidate agrees with the held one - but only count this as a
        # NEW confirmation if it's not just the fast resolve timer asking
        # again about the same unchanged proposal (see docstring). Exact
        # float equality is fine here: an unchanged source genuinely
        # returns the identical Python float, since neither depth_scale
        # nor a ToF median recomputes between ticks unless new underlying
        # data arrived.
        last_proposal = held.get("last_proposal")
        if last_proposal is not None and candidate == last_proposal:
            return None  # same proposal seen again on the fast timer - not a new confirmation

        held["last_proposal"] = candidate
        held["confirmations"] += 1
        if held["confirmations"] < constants.QUARANTINE_CONFIRMATIONS_REQUIRED:
            self.get_logger().info(
                f"map_id {map_id}: quarantined jump to {candidate:.6f} confirmed "
                f"({held['confirmations']}/{constants.QUARANTINE_CONFIRMATIONS_REQUIRED})."
            )
            return None

        # Confirmed enough distinct times in a row - this is a genuine,
        # sustained correction, not a single noisy cycle's outlier.
        # Adopt it and clear the quarantine entry.
        self.get_logger().warn(
            f"map_id {map_id}: scale factor jump to {candidate:.6f} confirmed "
            f"{held['confirmations']}x - adopting."
        )
        self._quarantine.pop(map_id, None)
        return candidate

    def _resolve_all_active_scales(self):
        """
            Fast-path re-resolve for every currently tracked map_id.
            Called by the resolve timer so that ToF's 10 Hz updates
            actually reach current_scale_factors in "auto"/"tof" mode
            rather than waiting until the next Depth-Anything result
            arrives (which can take 30-60s).
        """
        for map_id in list(self._map_state.keys()):
            self._resolve_active_scale(map_id)

    def cycle_scale_source_mode(self) -> str:
        """
            Cycles tof -> depth -> auto -> tof -> ...
            Called from main.py's 'm' key wiring via
            manual_control.on_scale_mode_cycle_requested.
            Returns the new mode string so the caller can update the HUD.
        """
        order = ["tof", "depth", "auto"]
        current = self.scale_source_mode
        idx = order.index(current) if current in order else 0
        self.scale_source_mode = order[(idx + 1) % len(order)]
        self.get_logger().info(
            f"ScaleFactorManager: scale source mode -> '{self.scale_source_mode}'"
        )
        # Immediately re-resolve all active map_ids so the switch takes
        # effect without waiting for the next sensor tick.
        self._resolve_all_active_scales()
        return self.scale_source_mode
    # ****************************************************************************************

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
        if self._resolve_timer is not None:
            self._resolve_timer.cancel()

        self._request_queue.put(None)  # sentinel - see depth_inference_worker.run_worker
        self._worker_process.join(timeout=5.0)
        if self._worker_process.is_alive():
            self.get_logger().warn("Depth inference worker didn't exit cleanly - terminating.")
            self._worker_process.terminate()

        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
