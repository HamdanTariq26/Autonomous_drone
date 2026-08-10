# Published once per detected SLAM map re-init by perception/live_scaler.py.
# Translation is already scaled by the new map_id's scale factor (metric units)
# so consumers apply it directly to new-map metric points/poses without further
# scaling. Rotation is scale-invariant and applied as-is.
#
# accepted=false means the sanity bounds rejected the alignment (offset or gap
# too large) and the subscriber should fall back to a hard OcTree reset instead
# of applying the transform. offset_m and gap_sec are always populated for
# diagnostic logging on the subscriber side.
int64 old_map_id
int64 new_map_id
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
bool accepted
float64 offset_m
float64 gap_sec
