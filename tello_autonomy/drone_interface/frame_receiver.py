"""
drone_interface/frame_receiver.py

Single job: hand back the latest camera frame as a BGR numpy array.
Depends only on drone_interface.tello_driver.TelloDriver (specifically
its frame_reader, set up during connect()). No commands sent, no lock
needed - reading frame_reader.frame is safe from any thread since
djitellopy's background thread overwrites it in place rather than
queueing.

djitellopy hands back frames in RGB order (via PyAV -> PIL), but
OpenCV/cv_bridge/ORB-SLAM3 all expect BGR. The conversion happens once,
here, so every caller downstream gets correct BGR without needing to
remember to convert it themselves.
"""

import cv2


class FrameReceiver:
    """
        Wraps a TelloDriver's frame_reader. Ask it for frames with
        get_frame_bgr() - that's the only method callers should use.
    """

    def __init__(self, tello_driver):
        """
            tello_driver: a drone_interface.tello_driver.TelloDriver
                          instance that has already had connect() called
                          on it (so tello_driver.frame_reader is set).
        """
        self._tello_driver = tello_driver

    # ****************************************************************************************
    def get_frame_bgr(self):
        """
            Returns the latest camera frame as a BGR numpy array, or
            None if no frame has arrived yet (e.g. connect() hasn't
            been called on the TelloDriver, or the stream hasn't
            produced a first frame yet).
        """
        frame_reader = self._tello_driver.frame_reader
        if frame_reader is None:
            return None
        frame_rgb = frame_reader.frame
        if frame_rgb is None:
            return None
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    # ****************************************************************************************