"""
Colour-blob detection for HW9 (NumPy + OpenCV only, no ROS).

Each cone colour is described by one or more HSV boxes in OpenCV units
(H 0-179, S 0-255, V 0-255), written as a flat list of 6 numbers per box:

    [h_lo, h_hi, s_lo, s_hi, v_lo, v_hi,  h_lo, h_hi, ...]

If h_lo > h_hi the hue range wraps through 0, which is what red needs.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

# Starting points only: lighting and the camera's white balance shift these,
# so measure your own cones with `cone_calib` and put the values in the
# params file.
DEFAULT_HSV = {
    'red':    [170, 6, 120, 255, 70, 255],
    'orange': [7, 20, 140, 255, 110, 255],
    'yellow': [21, 34, 110, 255, 110, 255],
    'green':  [35, 85, 80, 255, 40, 255],
    'blue':   [95, 125, 120, 255, 40, 255],
    'purple': [126, 150, 60, 255, 40, 255],
    'pink':   [151, 169, 60, 255, 100, 255],
}

# BGR colours used only for drawing the output images.
DRAW_BGR = {
    'red': (0, 0, 230), 'orange': (0, 130, 255), 'yellow': (0, 220, 240),
    'green': (40, 190, 40), 'blue': (230, 90, 20), 'purple': (180, 40, 140),
    'pink': (180, 105, 255),
}

_OPEN = np.ones((3, 3), np.uint8)
_CLOSE = np.ones((5, 5), np.uint8)


def draw_colour(name):
    return DRAW_BGR.get(name, (255, 255, 255))


def parse_boxes(flat, name='colour'):
    """[h_lo, h_hi, s_lo, s_hi, v_lo, v_hi, ...] -> list of 6-tuples."""
    vals = [int(v) for v in flat]
    if not vals or len(vals) % 6:
        raise ValueError(f'HSV range for {name!r} needs 6 numbers per box, got {len(vals)}')
    boxes = [tuple(vals[i:i + 6]) for i in range(0, len(vals), 6)]
    for h0, h1, s0, s1, v0, v1 in boxes:
        if not (0 <= h0 <= 179 and 0 <= h1 <= 179):
            raise ValueError(f'{name}: hue must be 0-179 (OpenCV units)')
        if not (0 <= s0 <= s1 <= 255 and 0 <= v0 <= v1 <= 255):
            raise ValueError(f'{name}: need 0 <= lo <= hi <= 255 for S and V')
    return boxes


def min_blob_area(width, height, fraction):
    return max(20, int(fraction * width * height))


def colour_mask(hsv, boxes):
    """Binary mask (uint8 0/255) of the pixels inside any of the HSV boxes."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for h0, h1, s0, s1, v0, v1 in boxes:
        spans = [(h0, h1)] if h0 <= h1 else [(h0, 179), (0, h1)]
        for a, b in spans:
            mask |= cv2.inRange(hsv, np.array([a, s0, v0], np.uint8),
                                np.array([b, s1, v1], np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _OPEN)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _CLOSE)
    return mask


@dataclass
class Blob:
    cx: float          # area-weighted centroid (pixels)
    cy: float
    area: int          # pixels
    x: int             # bounding box
    y: int
    w: int
    h: int

    @property
    def bottom(self):
        return self.y + self.h


def find_blob(mask, min_area):
    """
    Largest blob in the mask, or None if it is smaller than min_area.

    Traffic cones have white reflective bands, which split the colour into
    stacked pieces. Pieces directly above or below the largest one (they
    overlap it horizontally and are close vertically) are merged into it.
    """
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None
    main = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[main, cv2.CC_STAT_AREA] < min_area:
        return None
    mx, my, mw, mh = (int(v) for v in stats[main, :4])
    parts = [main]
    for j in range(1, n):
        if j == main or stats[j, cv2.CC_STAT_AREA] < max(4, 0.1 * min_area):
            continue
        x, y, w, h = (int(v) for v in stats[j, :4])
        overlap = min(mx + mw, x + w) - max(mx, x)
        vgap = max(y - (my + mh), my - (y + h))
        if overlap >= 0.5 * min(w, mw) and vgap <= max(4, 0.5 * mh):
            parts.append(j)
    a = stats[parts, cv2.CC_STAT_AREA].astype(float)
    cx = float(np.dot(a, cents[parts, 0]) / a.sum())
    cy = float(np.dot(a, cents[parts, 1]) / a.sum())
    x0 = int(stats[parts, 0].min())
    y0 = int(stats[parts, 1].min())
    x1 = int((stats[parts, 0] + stats[parts, 2]).max())
    y1 = int((stats[parts, 1] + stats[parts, 3]).max())
    return Blob(cx, cy, int(a.sum()), x0, y0, x1 - x0, y1 - y0)


def detect_all(bgr, table, min_area, want_masks=False):
    """Detect the largest blob of every colour in `table` {name: boxes}."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    dets, masks = {}, {}
    for name, boxes in table.items():
        m = colour_mask(hsv, boxes)
        dets[name] = find_blob(m, min_area)
        if want_masks:
            masks[name] = m
    return (dets, masks) if want_masks else dets


def pixel_offset(cx, width):
    """-1 (left edge) .. 0 (centre) .. +1 (right edge)."""
    return (cx - width / 2.0) / (width / 2.0)


def pixel_to_bearing(cx, width, hfov_rad):
    """Bearing of image column cx relative to the optical axis (rad, + = left)."""
    f = (width / 2.0) / math.tan(hfov_rad / 2.0)
    return math.atan2(width / 2.0 - cx, f)


# ---------------------------------------------------------------- ROS images
_RAW_CHANNELS = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1,
                 'yuv422': 2, 'uyvy': 2, 'yuv422_yuy2': 2, 'yuyv': 2}


def image_msg_to_bgr(msg):
    """sensor_msgs/Image -> BGR uint8 array (no cv_bridge needed)."""
    enc = msg.encoding.lower()
    ch = _RAW_CHANNELS.get(enc)
    if ch is None:
        raise ValueError(f'unsupported image encoding {msg.encoding!r}')
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    img = buf.reshape(h, msg.step)[:, :w * ch].reshape(h, w, ch)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc in ('yuv422', 'uyvy'):
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)
    if enc in ('yuv422_yuy2', 'yuyv'):
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    return img.copy()


def compressed_msg_to_bgr(msg):
    """sensor_msgs/CompressedImage (jpeg/png) -> BGR uint8 array."""
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


# ------------------------------------------------------------- calibration
def centre_hsv(bgr, half=8):
    """Median H, S, V of the (2*half+1)^2 patch at the image centre."""
    h, w = bgr.shape[:2]
    patch = bgr[h // 2 - half:h // 2 + half + 1, w // 2 - half:w // 2 + half + 1]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(float)
    # Hue is circular (0 == 180): take the median relative to the circular mean.
    ang = hsv[:, 0] * (2 * math.pi / 180.0)
    centre = math.atan2(np.sin(ang).mean(), np.cos(ang).mean()) * 180.0 / (2 * math.pi)
    rel = (hsv[:, 0] - centre + 90.0) % 180.0 - 90.0
    hue = int(round(centre + np.median(rel))) % 180
    return hue, int(np.median(hsv[:, 1])), int(np.median(hsv[:, 2]))


def suggest_box(h, s, v, h_tol=8):
    """A starting HSV box around a measured colour (hue may wrap)."""
    return [(h - h_tol) % 180, (h + h_tol) % 180,
            max(50, s - 70), 255, max(40, v - 90), 255]


# ----------------------------------------------------------------- drawing
def put_label(img, text, org, colour, scale=0.45, outline=True):
    if outline:                       # black edge keeps text readable on camera images
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv2.LINE_AA)


def annotate(bgr, dets, highlight=None, text=None):
    """Copy of the frame with every detection boxed and labelled."""
    img = bgr.copy()
    h, w = img.shape[:2]
    cv2.line(img, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)
    for name, b in dets.items():
        if b is None:
            continue
        col = draw_colour(name)
        cv2.rectangle(img, (b.x, b.y), (b.x + b.w, b.y + b.h), col, 3 if name == highlight else 1)
        cv2.circle(img, (int(b.cx), int(b.cy)), 3, col, -1)
        put_label(img, name, (b.x, max(12, b.y - 5)), col)
    if text:
        put_label(img, text, (4, h - 8), (255, 255, 255), 0.4)
    return img
