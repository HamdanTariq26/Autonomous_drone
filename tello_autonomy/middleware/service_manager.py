"""
middleware/service_manager.py

STUB - no real logic yet.

Nothing in this project currently uses ROS2 services - every
interaction with the C++ SLAM node is topic-based (see topic_manager.py
and ros_bridge.py). This file is reserved for when a genuine
request/response need shows up, e.g.:

  - "give me the current scale factor for this map_id" (perception/
    layer asking something for an on-demand value rather than waiting
    on a topic)
  - "save the map now" (an explicit one-shot command with a
    success/failure reply, as opposed to fire-and-forget)

Per the handoff doc (Section 7): don't invent service definitions
speculatively. Leave this as a stub until a concrete consumer actually
needs request/response semantics - adding unused .srv definitions and
wrapper code ahead of time is exactly the kind of speculative
complexity the rest of this package's layers avoid (see occupancy_map/,
exploration/, search/, goals/, which are NOT STARTED for the same
reason).

When a real need appears, follow the same pattern as topic_manager.py:
centralize service *creation* here (client + server setup, matched
QoS/timeout policy), rather than letting callers hand-roll
create_service()/create_client() calls inline.
"""

# No imports, no classes, no functions yet - intentionally empty beyond
# this docstring. See the docstring above for what belongs here once
# there's a real consumer.