"""OpenCV observation construction; uses the pinned, version-fingerprinted detector."""
import hashlib
import json
import math
import cv2
import numpy as np
from .geometry import opening_polygon


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def inner_opening(target, width, height):
    method = target['method']
    if method == 'nested_frame':
        contour = target.get('inner_contour')
        kind = 'contour_inner_opening'
    elif method in ('checker_texture','checker_contour') and target['outline_kind'] == 'opening':
        contour = target['contour']
        kind = 'approximate_inner_opening'
    else:
        raise ValueError('no_supported_inner_opening')
    if contour is None: raise ValueError('missing_inner_contour')
    # Order cyclically in the image. No physical TL/TR correspondence is claimed.
    points = np.asarray(contour,dtype=float).reshape(4,2)
    mid = np.mean(points,axis=0)
    order = np.argsort(np.arctan2(points[:,1]-mid[1],points[:,0]-mid[0]))
    pts = tuple(tuple(float(v) for v in p) for p in points[order])
    center, _ = opening_polygon(pts,width,height)
    return center,pts,kind


def camera_rays(info, frame, width, height, stamp, max_age=0.25):
    if info is None: raise ValueError('missing_camera_info')
    if info.header.frame_id != frame or not frame: raise ValueError('camera_info_optical_frame')
    if (info.width,info.height) != (width,height): raise ValueError('camera_info_dimensions')
    if info.header.stamp.to_sec() <= 0 or abs(info.header.stamp.to_sec()-stamp) > max_age: raise ValueError('camera_info_capture_age')
    if info.distortion_model not in ('plumb_bob','rational_polynomial'): raise ValueError('unsupported_distortion')
    if info.binning_x not in (0,1) or info.binning_y not in (0,1): raise ValueError('camera_binning_unverified')
    if any((info.roi.x_offset,info.roi.y_offset,info.roi.width,info.roi.height)): raise ValueError('camera_roi_unverified')
    k, d = np.asarray(info.K,dtype=float),np.asarray(info.D,dtype=float)
    if k.shape != (9,) or len(d) not in (4,5,8,12,14) or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)):
        raise ValueError('nonfinite_or_incomplete_intrinsics')
    if k[0] <= 0 or k[4] <= 0 or not (0 <= k[2] < width and 0 <= k[5] < height): raise ValueError('invalid_intrinsics')
    if abs(k[8]-1) > 1e-9 or any(abs(k[i]) > 1e-9 for i in (1,3,6,7)): raise ValueError('unsupported_intrinsic_matrix')
    meta = dict(width=width,height=height,frame=frame,model=info.distortion_model,K=k.tolist(),D=d.tolist())
    return k.reshape(3,3),d,digest(meta)


def undistorted_rays(center, corners, k, d):
    pts = np.asarray([center]+list(corners),dtype=np.float64).reshape(-1,1,2)
    normed = cv2.undistortPoints(pts,k,d).reshape(5,2)
    if not np.all(np.isfinite(normed)): raise ValueError('undistortion_failed')
    return [(float(x),float(y),1.) for x,y in normed]
