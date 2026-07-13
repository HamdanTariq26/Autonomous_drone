"""
perception/depth/depth_anything_v2.py

Model loading + inference ONLY. This is the one file in the whole
project allowed to `import torch` / `depth_anything_v2` - see handoff
doc, Section 7. No CSV reading, no keyframe matching, no ROS2 - just
"give me a BGR image, get back a metric depth map."

The actual Depth-Anything-V2 metric_depth repo lives outside this
project (see config.constants.DEPTH_ANYTHING_V2_REPO_PATH) - it is
added to sys.path below rather than pip-installed, so its
`depth_anything_v2` package can be imported directly.

Loaded model is cached as a module-level singleton: the first call to
load_model() (or an infer_metric_depth() call with no model passed in)
loads it once; later calls with the same settings reuse it rather than
reloading the checkpoint from disk every time. This matters because
perception/scale_factor_manager.py may call infer_metric_depth() many
times per recompute, across many recomputes over a flight.
"""

import sys

from config import constants

if constants.DEPTH_ANYTHING_V2_REPO_PATH not in sys.path:
    sys.path.insert(0, constants.DEPTH_ANYTHING_V2_REPO_PATH)

import torch  # noqa: E402  (import after sys.path insert, deliberately)
from depth_anything_v2.dpt import DepthAnythingV2  # noqa: E402


# Model config per encoder size - vits is the practical choice for CPU
# inference on this project's 8GB RAM machine (see handoff doc, Section
# 8). See the Depth-Anything-V2 repo README for the other variants.
MODEL_CONFIGS = {
    "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
    "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
    "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
}

# Module-level singleton - see docstring above.
_model = None
_model_config_key = None


def load_model(checkpoint_path=None, encoder=None, max_depth=None, device=None):
    """
        Loads (or returns the already-loaded) Depth Anything V2 metric
        model. All four args default to config.constants values -
        pass an override only for a specific one-off need (e.g. trying
        a different checkpoint interactively).

        max_depth=20.0 (the constants default) matches the indoor/
        Hypersim-trained checkpoint's training range. Use max_depth=80.0
        with a vkitti (outdoor) checkpoint instead, if this project ever
        flies outside.
    """
    global _model, _model_config_key

    checkpoint_path = checkpoint_path or constants.DEPTH_ANYTHING_V2_CHECKPOINT_PATH
    encoder = encoder or constants.DEPTH_ANYTHING_V2_ENCODER
    max_depth = max_depth if max_depth is not None else constants.DEPTH_ANYTHING_V2_MAX_DEPTH
    device = device or constants.DEPTH_ANYTHING_V2_DEVICE

    config_key = (checkpoint_path, encoder, max_depth, device)
    if _model is not None and _model_config_key == config_key:
        return _model  # already loaded with these exact settings - reuse it

    cfg = MODEL_CONFIGS[encoder]
    model = DepthAnythingV2(**cfg, max_depth=max_depth)
    model.load_state_dict(torch.load(checkpoint_path, map_location="cpu"))
    model = model.to(device).eval()

    _model = model
    _model_config_key = config_key
    return _model


def infer_metric_depth(image_bgr, model=None):
    """
        Runs metric depth inference on a single BGR image (as returned
        by drone_interface.frame_receiver.FrameReceiver.get_frame_bgr()
        or cv2.imread()). Returns an (H, W) numpy array of metric depth
        in meters.

        If model is None (the common case - callers in
        scale_factor_manager.py don't manage a model instance
        themselves), load_model() is called with default settings,
        loading once and reusing the cached singleton on later calls.
    """
    if model is None:
        model = load_model()
    with torch.no_grad():
        return model.infer_image(image_bgr)
