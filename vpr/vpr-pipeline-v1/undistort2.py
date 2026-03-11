import cv2
import numpy as np
import argparse
import os
import sys

# 相机内参
fx = 1012.5934446010
fy = 1012.7695042461
cx = 959.1535002056
cy = 538.7995847097

# 畸变参数
k1 = 0.3278033404
k2 = -0.1678714011
p1 = 0.0000012166
p2 = 0.0000388473
k3 = -0.0124574817
k4 = 0.7233630949
k5 = -0.1359816721
k6 = -0.0630068024

# 构建内参矩阵和畸变系数
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float64)
dist_coeffs = np.array([k1, k2, p1, p2, k3, k4, k5, k6], dtype=np.float64)

def main():
    parser = argparse.ArgumentParser(description='Undistort an image using provided camera intrinsics.')
    parser.add_argument('input', help='input image path')
    parser.add_argument('-o', '--output', help='output image path (default: undistorted_<input>)', default=None)
    args = parser.parse_args()

    img = cv2.imread(args.input)
    if img is None:
        print(f'Error: failed to read image: {args.input}', file=sys.stderr)
        sys.exit(1)

    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)

    if args.output:
        out_path = args.output
    else:
        out_dir = os.path.dirname(args.input) or '.'
        out_name = 'undistorted_' + os.path.basename(args.input)
        out_path = os.path.join(out_dir, out_name)

    if not cv2.imwrite(out_path, undistorted_img):
        print(f'Error: failed to write image: {out_path}', file=sys.stderr)
        sys.exit(1)

    print(f'Saved undistorted image to: {out_path}')


if __name__ == '__main__':
    main()