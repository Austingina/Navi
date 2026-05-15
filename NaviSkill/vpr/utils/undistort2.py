"""PINHOLE camera undistortion (OpenCV rational distortion model).

Mirrors the API of :class:`undistort.PolyFisheyeUndistorter` for cameras
modelled by the standard OpenCV pinhole + 8-coefficient rational
distortion (``[k1, k2, p1, p2, k3, k4, k5, k6]``). The undistortion map
is built once via ``cv2.initUndistortRectifyMap`` and cached per input
image size, so ``undistort()`` only pays for a ``cv2.remap`` call.

Usage::

    undistorter = PinHoleUndistorter.from_defaults()
    perspective_img = undistorter.undistort(raw_img)
    # or, for ROS CompressedImage / JPEG bytes:
    rgb = undistorter.undistort_bytes(jpeg_bytes)

Also provides a small CLI: ``python undistort2.py <input> [-o out]``.
"""

import argparse
import os
import sys

import cv2
import numpy as np


class PinHoleUndistorter:
    """Undistort PINHOLE images using OpenCV's rational distortion model.

    Parameters
    ----------
    camera_matrix : (3, 3) array_like
        Intrinsic matrix ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``.
    dist_coeffs : (N,) array_like
        OpenCV distortion vector. Typically the 8-parameter rational
        model ``[k1, k2, p1, p2, k3, k4, k5, k6]``; the 5/12/14-parameter
        variants accepted by OpenCV are also supported.
    image_size : (w, h) tuple, optional
        If given, the remap table is precomputed at construction time
        for this input size. Otherwise the map is built lazily on the
        first :meth:`undistort` / :meth:`undistort_bytes` call and
        cached for that size; a different size triggers a one-time
        rebuild.
    """

    def __init__(self, camera_matrix, dist_coeffs, image_size=None):
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        # 每个输入尺寸单独缓存 (map_x, map_y)；R1Pro 部署里只会有一个尺寸
        self._maps: dict = {}
        if image_size is not None:
            self._build_maps(int(image_size[0]), int(image_size[1]))

    def _build_maps(self, w, h):
        """构建并缓存指定输入尺寸的 remap 表（无 alpha 调整，等价 cv2.undistort）。"""
        map_x, map_y = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            R=None,
            newCameraMatrix=self.camera_matrix,
            size=(w, h),
            m1type=cv2.CV_32FC1,
        )
        self._maps[(w, h)] = (map_x, map_y)
        return map_x, map_y

    def _maps_for(self, w, h):
        m = self._maps.get((w, h))
        if m is None:
            m = self._build_maps(w, h)
        return m

    def undistort(self, img):
        """Undistort a BGR/RGB ``numpy`` image. Color space is preserved."""
        h, w = img.shape[:2]
        map_x, map_y = self._maps_for(w, h)
        return cv2.remap(
            img, map_x, map_y, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0,
        )

    def undistort_bytes(self, jpeg_bytes):
        """Decode JPEG bytes, undistort, return **RGB** ``numpy`` array.

        The RGB return shape matches :meth:`PolyFisheyeUndistorter.undistort_bytes`,
        so call sites that hand the result to ``Image.fromarray(...)`` /
        ``undistort_and_save`` work uniformly across both undistorters.
        """
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("PinHoleUndistorter: cv2.imdecode failed (corrupt JPEG?)")
        undist = self.undistort(img)
        return cv2.cvtColor(undist, cv2.COLOR_BGR2RGB)

    # ------------------------------------------------------------------
    # Factory: known calibrations
    # ------------------------------------------------------------------

    @classmethod
    def from_defaults(cls, image_size=None):
        """Return an undistorter configured for the R1Pro front camera.

        Intrinsics and 8-coefficient rational distortion were calibrated
        from the on-board R1Pro left/front sensor; see commit history
        for the original calibration run.
        """
        camera_matrix = np.array([
            [1012.5934446010, 0,                959.1535002056],
            [0,               1012.7695042461,  538.7995847097],
            [0,               0,                1.0],
        ], dtype=np.float64)
        dist_coeffs = np.array([
            0.3278033404,    # k1
            -0.1678714011,   # k2
            0.0000012166,    # p1
            0.0000388473,    # p2
            -0.0124574817,   # k3
            0.7233630949,    # k4
            -0.1359816721,   # k5
            -0.0630068024,   # k6
        ], dtype=np.float64)
        return cls(camera_matrix, dist_coeffs, image_size=image_size)


# ---------------------------------------------------------------------------
# CLI: keep the original one-shot utility behaviour
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Undistort a pinhole image using the R1Pro defaults.",
    )
    parser.add_argument("input", help="input image path")
    parser.add_argument("-o", "--output",
                        help="output image path (default: undistorted_<input>)",
                        default=None)
    args = parser.parse_args()

    img = cv2.imread(args.input)
    if img is None:
        print(f"Error: failed to read image: {args.input}", file=sys.stderr)
        sys.exit(1)

    undistorter = PinHoleUndistorter.from_defaults()
    undistorted_img = undistorter.undistort(img)

    if args.output:
        out_path = args.output
    else:
        out_dir = os.path.dirname(args.input) or "."
        out_name = "undistorted_" + os.path.basename(args.input)
        out_path = os.path.join(out_dir, out_name)

    if not cv2.imwrite(out_path, undistorted_img):
        print(f"Error: failed to write image: {out_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Saved undistorted image to: {out_path}")


if __name__ == "__main__":
    main()
