"""Narrow metadata repair for the inspected legacy astra RAW-depth driver.

Its onNewFrameCallback stamps CameraInfo but omits the raw depth frame_id.
Only repair that metadata after factory K + optical-frame + profile checks.
Never re-label a depth image or claim hardware D2C registration succeeded.
"""
import math
import numpy as np
from .geometry import finite, quaternion_matrix


def factory_checked_raw_info(rgb_info, depth_info, depth_frame, rgb_frame, params,
                             raw_depth_align, expected_prefix='camera'):
    if raw_depth_align is not False:
        raise ValueError('astra_raw_contract_requires_depth_align_false')
    if depth_frame != expected_prefix+'_depth_optical_frame' or rgb_frame != expected_prefix+'_color_optical_frame':
        raise ValueError('astra_unexpected_raw_optical_frames')
    if depth_info is None or rgb_info is None or not params.get('success', False):
        raise ValueError('astra_factory_calibration_unavailable')
    if depth_info['frame_id'] not in ('', depth_frame) or rgb_info['frame_id'] != rgb_frame:
        raise ValueError('astra_camera_info_frame_conflict')
    if depth_info['width'] != 640 or depth_info['height'] not in (400, 480) or (rgb_info['width'], rgb_info['height']) != (640, 480):
        raise ValueError('astra_factory_profile_requires_native_raw640x400_or_480_rgb640x480')
    for info, key in ((depth_info, 'l_intr_p'), (rgb_info, 'r_intr_p')):
        fx, fy, cx, cy = finite(params[key], (4,))
        expected = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
        actual = finite(info['K']).reshape(3, 3)
        if min(fx, fy) < 100 or not np.allclose(actual, expected, atol=.5, rtol=0):
            raise ValueError('astra_published_intrinsics_disagree_with_factory')
    rotation = finite(params['r2l_r']).reshape(3, 3)
    baseline = finite(params['r2l_t'], (3,))*.001
    if not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=.005) or abs(np.linalg.det(rotation)-1) > .005:
        raise ValueError('astra_factory_rotation_invalid')
    if not .001 <= np.linalg.norm(baseline) <= .25:
        raise ValueError('astra_factory_baseline_invalid')
    result = dict(depth_info)
    result['frame_id'] = depth_frame
    result['metadata_repair'] = 'missing_raw_depth_camera_info_frame_only_factory_K_checked'
    result['factory_intrinsics_checked'] = True
    result['distortion_and_temporal_accuracy_flight_verified'] = False
    return result

def factory_checked_tf(rotation, translation, params):
    """Check against the EXACT inspected legacy driver's static TF construction.

    This is a consistency check, NOT independent extrinsic calibration. That
    driver transposes the documented row-major matrix and inverts translation
    after changing rotation coordinates. Do not silently fix just one side or
    assume this convention measures the physical sensor correctly.
    """
    raw = finite(params['r2l_r']).reshape(3, 3)
    shift = finite(params['r2l_t'], (3,))*.001
    optical = quaternion_matrix([-.5, .5, -.5, .5])
    q = optical.dot(raw.T).dot(optical.T)
    expected_r = raw.T
    expected_t = optical.T.dot(q).dot(optical).dot(q.T).dot(shift)
    if (not np.allclose(finite(rotation, (3, 3)), expected_r, atol=2e-4, rtol=0) or
            not np.allclose(finite(translation, (3,)), expected_t, atol=.0005, rtol=0)):
        raise ValueError('tf_disagrees_with_inspected_legacy_factory_chain')
    return dict(extrinsic_check='legacy_driver_consistency_only', physical_extrinsics_verified=False)
