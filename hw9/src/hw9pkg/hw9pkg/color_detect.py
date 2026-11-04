"""
HW9: find coloured traffic cones in a camera image by HSV thresholding.

Pure OpenCV / NumPy (no ROS), so it also works on saved pictures:

  ros2 run hw9pkg calibrate detect hw9_output/snap_01.png

OpenCV's HSV: hue 0-179 (half degrees), saturation and value 0-255.
Each colour is one range (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi). Red sits at
both ends of the hue circle, so a range with h_lo > h_hi wraps around:
(170, ..., 6, ...) means hue 170-179 or 0-6.

The defaults below are starting points for bright plastic cones. Lab
lighting moves them, so tune yours with `calibrate tune <colour>`, which
saves hsv_ranges.yaml; cone_finder reads that file when it exists.
"""

import os
from dataclasses import dataclass

import cv2
import numpy as np
import yaml

DEFAULT_HSV = {
    'red':    (170, 120, 70, 6, 255, 255),
    'orange': (7, 140, 100, 20, 255, 255),
    'yellow': (21, 110, 110, 35, 255, 255),
    'green':  (40, 80, 50, 85, 255, 255),
    'blue':   (95, 120, 50, 130, 255, 255),
    'purple': (131, 60, 50, 165, 255, 255),
}

DEFAULT_HSV_FILE = 'hsv_ranges.yaml'

# BGR colours used to draw each cone's box in the debug images
DRAW_BGR = {
    'red': (0, 0, 255), 'orange': (0, 140, 255), 'yellow': (0, 230, 255),
    'green': (0, 200, 0), 'blue': (255, 80, 0), 'purple': (200, 0, 160),
}


@dataclass
class Blob:
    color: str
    u: float          # centroid column (px), 0 = left edge
    v: float          # centroid row (px), 0 = top edge
    area: float       # contour area (px^2)
    bbox: tuple       # (x, y, w, h) in px


# ---------------------------------------------------------------------- #
# HSV table
# ---------------------------------------------------------------------- #
def load_hsv(path=None):
    """Default table, overridden by the YAML file if it exists. Returns (table, source)."""
    table = dict(DEFAULT_HSV)
    path = path or DEFAULT_HSV_FILE
    if not os.path.isfile(path):
        return table, 'built-in defaults'
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    for color, rng in data.items():
        rng = tuple(int(v) for v in rng)
        if len(rng) != 6:
            raise ValueError(f'{path}: {color} needs 6 numbers, got {rng}')
        table[str(color).lower()] = rng
    return table, os.path.abspath(path)


def save_hsv(path, color, rng):
    """Write one colour's range into the YAML file, keeping the others."""
    data = {}
    if os.path.isfile(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    data[color] = [int(v) for v in rng]
    with open(path, 'w') as f:
        f.write('# h_lo, s_lo, v_lo, h_hi, s_hi, v_hi  (OpenCV HSV; h_lo > h_hi wraps past 179)\n')
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=True)


# ---------------------------------------------------------------------- #
# Detection
# ---------------------------------------------------------------------- #
def to_hsv(bgr):
    return cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)


def color_mask(hsv, rng):
    """Binary mask (0/255) of pixels inside one HSV range, with specks removed."""
    h_lo, s_lo, v_lo, h_hi, s_hi, v_hi = rng
    lo, hi = np.array([h_lo, s_lo, v_lo], np.uint8), np.array([h_hi, s_hi, v_hi], np.uint8)
    if h_lo <= h_hi:
        mask = cv2.inRange(hsv, lo, hi)
    else:   # wraps around hue 179 -> 0 (red)
        mask = (cv2.inRange(hsv, lo, np.array([179, s_hi, v_hi], np.uint8))
                | cv2.inRange(hsv, np.array([0, s_lo, v_lo], np.uint8), hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return mask


def largest_blob(mask, color, min_area):
    """Largest connected region of the mask that could be a cone, or None."""
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w > 3 * h:     # a flat stripe (floor tape, a shelf edge), not a cone
            continue
        if best is None or area > best.area:
            m = cv2.moments(c)
            u = m['m10'] / m['m00'] if m['m00'] else x + w / 2
            v = m['m01'] / m['m00'] if m['m00'] else y + h / 2
            best = Blob(color, u, v, area, (x, y, w, h))
    return best


def detect(bgr, colors, table, min_area_frac=0.001):
    """{colour: Blob} for every colour in `colors` that is visible."""
    hsv = to_hsv(bgr)
    min_area = min_area_frac * bgr.shape[0] * bgr.shape[1]
    found = {}
    for color in colors:
        blob = largest_blob(color_mask(hsv, table[color]), color, min_area)
        if blob is not None:
            found[color] = blob
    return found


def median_hsv(bgr, u, v, half=8):
    """Median (h, s, v) of a small square around pixel (u, v), for calibration."""
    hsv = to_hsv(bgr)
    rows, cols = hsv.shape[:2]
    patch = hsv[max(0, v - half):min(rows, v + half + 1), max(0, u - half):min(cols, u + half + 1)]
    return tuple(int(x) for x in np.median(patch.reshape(-1, 3), axis=0))


def annotate(bgr, blobs, title=''):
    """Copy of the image with each blob boxed and labelled."""
    out = bgr.copy()
    for blob in blobs.values():
        x, y, w, h = blob.bbox
        col = DRAW_BGR.get(blob.color, (255, 255, 255))
        cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)
        cv2.circle(out, (int(blob.u), int(blob.v)), 3, col, -1)
        cv2.putText(out, blob.color, (x, max(10, y - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, col, 1, cv2.LINE_AA)
    cols = out.shape[1]
    cv2.line(out, (cols // 2, 0), (cols // 2, out.shape[0]), (255, 255, 255), 1)
    if title:
        cv2.putText(out, title, (4, out.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (255, 255, 255), 1, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------- #
# ROS image message -> OpenCV BGR array (no cv_bridge needed)
# ---------------------------------------------------------------------- #
_CHANNELS = {'bgr8': 3, 'rgb8': 3, '8uc3': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1, '8uc1': 1}
_TO_BGR = {'rgb8': cv2.COLOR_RGB2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
           'rgba8': cv2.COLOR_RGBA2BGR, 'mono8': cv2.COLOR_GRAY2BGR, '8uc1': cv2.COLOR_GRAY2BGR}


def image_from_msg(msg):
    """sensor_msgs/Image or CompressedImage -> HxWx3 uint8 BGR array."""
    if hasattr(msg, 'format'):                       # CompressedImage (jpeg / png)
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f'could not decode compressed image ({msg.format})')
        return img
    enc = msg.encoding.lower()
    if enc not in _CHANNELS:
        raise ValueError(f'unsupported image encoding {msg.encoding!r}')
    n = _CHANNELS[enc]
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * n].reshape(msg.height, msg.width, n)
    if enc in _TO_BGR:
        img = cv2.cvtColor(img, _TO_BGR[enc])
    return np.ascontiguousarray(img)
