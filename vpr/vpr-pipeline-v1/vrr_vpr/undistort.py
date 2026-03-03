"""POLYFISHEYE camera undistortion.

Converts fisheye images to perspective projection using a polynomial
radial distortion model. The remap table is built once and reused.
"""

import cv2
import numpy as np


class PolyFisheyeUndistorter:
    """Undistort POLYFISHEYE images to perspective projection.

    Model: r(theta) = theta + k2*t^2 + ... + k7*t^7
    """

    def __init__(self, k2, k3, k4, k5, k6, k7,
                 A11, A12, A22, u0, v0,
                 out_w=1024, out_h=768, fov_deg=90):
        f_out = out_w / (2 * np.tan(np.radians(fov_deg) / 2))
        cx, cy = out_w / 2.0, out_h / 2.0

        uu, vv = np.meshgrid(
            np.arange(out_w, dtype=np.float64),
            np.arange(out_h, dtype=np.float64),
        )
        x_n = (uu - cx) / f_out
        y_n = (vv - cy) / f_out

        theta = np.arctan(np.sqrt(x_n ** 2 + y_n ** 2))
        phi = np.arctan2(y_n, x_n)

        coeffs = [0, 0, k2, k3, k4, k5, k6, k7]
        r = theta.copy()
        for i in range(2, 8):
            r += coeffs[i] * (theta ** i)

        x_d = r * np.cos(phi)
        y_d = r * np.sin(phi)

        self.map_x = (A11 * x_d + A12 * y_d + u0).astype(np.float32)
        self.map_y = (A22 * y_d + v0).astype(np.float32)
        self.out_w = out_w
        self.out_h = out_h

    def undistort(self, img):
        """Undistort a BGR/RGB image (numpy array) to perspective projection."""
        return cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def undistort_bytes(self, jpeg_bytes):
        """Decode JPEG bytes, undistort, return RGB numpy array."""
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        undist = self.undistort(img)
        return cv2.cvtColor(undist, cv2.COLOR_BGR2RGB)

    @classmethod
    def from_defaults(cls, out_w=1024, out_h=768, fov_deg=90):
        """Create with SHARE SLAM S20 left camera defaults."""
        return cls(
            k2=5.1509512746715336e-02, k3=-1.7864596798669050e-01,
            k4=3.4587287372421088e-01, k5=-3.0853738072040005e-01,
            k6=1.3690448456146820e-01, k7=-2.2977364891473839e-02,
            A11=1.4642282015861940e+03, A12=-1.5105701072420728e-02,
            A22=1.4638551370234854e+03,
            u0=1.7930402453626104e+03, v0=2.2626076145032089e+03,
            out_w=out_w, out_h=out_h, fov_deg=fov_deg,
        )

    @classmethod
    def from_yaml(cls, yaml_path, camera="fisheye_left", out_w=1024, out_h=768, fov_deg=90):
        """Load calibration from a SHARE SLAM S20 calibration.yaml file."""
        import re

        with open(yaml_path) as f:
            text = f.read()

        section_pattern = rf"{camera}:.*?maxIncidentAngle:\s*[\d.]+"
        match = re.search(section_pattern, text, re.DOTALL)
        if not match:
            raise ValueError(f"Camera '{camera}' not found in {yaml_path}")
        section = match.group()

        def extract(key):
            m = re.search(rf"{key}:\s*([-\d.eE+]+)", section)
            if not m:
                raise ValueError(f"Key '{key}' not found in {camera} section")
            return float(m.group(1))

        return cls(
            k2=extract("k2"), k3=extract("k3"), k4=extract("k4"),
            k5=extract("k5"), k6=extract("k6"), k7=extract("k7"),
            A11=extract("A11"), A12=extract("A12"), A22=extract("A22"),
            u0=extract("u0"), v0=extract("v0"),
            out_w=out_w, out_h=out_h, fov_deg=fov_deg,
        )
