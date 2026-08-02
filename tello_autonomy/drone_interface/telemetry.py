"""
drone_interface/telemetry.py

Read-only observer of drone state (battery, height, temperature,
flight time, ...). Depends only on drone_interface.tello_driver -
never sends a command, so it never needs tello_driver.cmd_lock. All
reads here come from djitellopy's cached state (populated by the
drone's continuous background state broadcast), not from actively
querying the drone.

Telemetry is opt-in, both as a whole and per-field:
  - `enabled=False` (whole monitor off) - read() returns {}, no field
    reads happen at all. Useful when telemetry logging isn't wanted
    for a given run and shouldn't add any overhead.
  - `fields=[...]` - choose exactly which fields matter. Defaults to a
    minimal, generally-useful set (battery, height, flight_time) rather
    than reading everything djitellopy exposes - extra fields nobody
    asked for are just noise. Pass ALL_FIELDS to enable everything.

This is deliberately a plain read() method rather than a background
logging thread - callers (a ROS2 node, a manual-flight script, a
future battery_watchdog in safety/) decide their own polling cadence
and what to do with the values. Keeping this passive/stateless makes
it trivial to reuse in different contexts without dragging along
threading or logging-format decisions it shouldn't be making.
"""


def _read_battery(tello):
    return tello.get_battery()


def _read_height(tello):
    return tello.get_height()


def _read_temperature(tello):
    return tello.get_temperature()


def _read_flight_time(tello):
    return tello.get_flight_time()


def _read_barometer(tello):
    return tello.get_barometer()


def _read_speed(tello):
    return {
        "speed_x": tello.get_speed_x(),
        "speed_y": tello.get_speed_y(),
        "speed_z": tello.get_speed_z(),
    }


def _read_tof(tello):
    # Returns the downward Time-of-Flight distance in cm from the Tello's
    # background state UDP stream.  This is the raw sensor output - it
    # includes the near-field clamp (~10 cm when <30 cm true distance) and
    # the out-of-range sentinel (~8192).  Validity filtering is the job of
    # perception/tof_scale_estimator.py, not here.
    return tello.get_distance_tof()


# Maps field name -> reader function. Add new fields here as djitellopy
# exposes them; nothing else needs to change to support a new field.
_FIELD_READERS = {
    "battery": _read_battery,
    "height": _read_height,
    "temperature": _read_temperature,
    "flight_time": _read_flight_time,
    "barometer": _read_barometer,
    "speed": _read_speed,
    "tof": _read_tof,
}

ALL_FIELDS = list(_FIELD_READERS.keys())
DEFAULT_FIELDS = ["battery", "height", "flight_time"]


class TelemetryMonitor:
    """
        Read-only telemetry. enable()/disable() can flip this on or off
        at runtime; read() returns {} whenever disabled, so callers
        don't need to check enabled themselves before calling read().
    """

    def __init__(self, tello_driver, enabled=True, fields=None):
        """
            tello_driver: a drone_interface.tello_driver.TelloDriver
                          instance, already connected.
            enabled: whether read() does anything at all.
            fields: list of field names from ALL_FIELDS to read.
                    Defaults to DEFAULT_FIELDS if not given.
        """
        self._tello_driver = tello_driver
        self.enabled = enabled

        fields = fields if fields is not None else DEFAULT_FIELDS
        unknown = set(fields) - set(ALL_FIELDS)
        if unknown:
            raise ValueError(f"Unknown telemetry field(s): {unknown}. Valid fields: {ALL_FIELDS}")
        self.fields = list(fields)

    # ****************************************************************************************
    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
    # ****************************************************************************************

    # ****************************************************************************************
    def read(self):
        """
            Returns a dict of {field_name: value} for every enabled
            field. Returns {} if this monitor is disabled. A field that
            fails to read (e.g. not supported by this firmware/SDK
            version) is silently omitted rather than raising - a
            telemetry hiccup should never be able to crash a flight.
        """
        if not self.enabled:
            return {}

        tello = self._tello_driver.tello
        result = {}
        for field in self.fields:
            reader = _FIELD_READERS[field]
            try:
                value = reader(tello)
            except Exception:
                continue  # field unsupported or transient read error - skip, don't crash
            if isinstance(value, dict):
                result.update(value)
            else:
                result[field] = value
        return result
    # ****************************************************************************************