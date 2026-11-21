"""Image-relative frame detection. Python 3.8 / OpenCV 4 compatible.

The observable is a four-sided opening in a textured or solid border. Pixel
size and perspective are measured anew, not checked against a metric survey.
Thresholds below express evidence quality/resolution, not known window size.
"""
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    corners: np.ndarray
    center: np.ndarray
    score: float
    method: str
    sides: list

    def as_dict(self):
        q = self.corners
        return dict(corners_px=q.tolist(), center_px=self.center.tolist(),
                    apparent_width_px=float((np.linalg.norm(q[1]-q[0])+np.linalg.norm(q[2]-q[3]))/2),
                    apparent_height_px=float((np.linalg.norm(q[3]-q[0])+np.linalg.norm(q[2]-q[1]))/2),
                    score=float(self.score), method=self.method, side_evidence=self.sides,
                    geometry_kind='observed_inner_quad_approximation')


def order_quad(points):
    q = np.asarray(points, np.float32).reshape(4, 2)
    center = q.mean(axis=0)
    q = q[np.argsort(np.arctan2(q[:, 1]-center[1], q[:, 0]-center[0]))]
    q = np.roll(q, -int(np.argmin(q.sum(axis=1))), axis=0)
    return q


def quad_center(q):
    # Intersection of diagonals = projective image of a rectangular opening's
    # physical center. An arithmetic average is biased under perspective.
    a = np.column_stack((q[2]-q[0], -(q[3]-q[1])))
    uv = np.linalg.solve(a, q[1]-q[0])
    return q[0]+uv[0]*(q[2]-q[0])


def valid_quad(q, shape):
    h, w = shape[:2]
    margin = max(2., .008*min(h, w))
    if not np.isfinite(q).all() or not cv2.isContourConvex(q.astype(np.float32)):
        return False
    if np.any(q < margin) or np.any(q[:, 0] > w-1-margin) or np.any(q[:, 1] > h-1-margin):
        return False
    area = abs(cv2.contourArea(q))
    sides = np.roll(q, -1, axis=0)-q
    lens = np.linalg.norm(sides, axis=1)
    if not .012*h*w <= area <= .88*h*w or lens.min() < .065*min(h, w):
        return False
    if lens.max()/lens.min() > 6.0:
        return False
    for i in range(4):
        cosine = abs(float(np.dot(sides[i], sides[i-1])/(lens[i]*lens[i-1])))
        if cosine > .8:
            return False
    return True


def contour_quad(contour, shape):
    hull = cv2.convexHull(contour)
    area = abs(cv2.contourArea(hull))
    if area < .012*shape[0]*shape[1] or abs(cv2.contourArea(contour)) < .60*area:
        return None
    perimeter = cv2.arcLength(hull, True)
    for eps in (.012, .025, .045):
        approx = cv2.approxPolyDP(hull, eps*perimeter, True)
        if len(approx) == 4:
            q = order_quad(approx)
            if valid_quad(q, shape):
                return q
    return None


def polygon_iou(a, b):
    try:
        overlap, _ = cv2.intersectConvexConvex(a.astype(np.float32), b.astype(np.float32))
        return float(overlap / max(1., abs(cv2.contourArea(a))+abs(cv2.contourArea(b))-overlap))
    except cv2.error:
        return 0.


def sample_band(array, q, side, start, end, rows=12, cols=80):
    # Width is a fraction of observed opening height/width; positive is outside.
    canonical = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    along = np.linspace(.08, .92, cols)
    across = np.linspace(start, end, rows)
    x, v = np.meshgrid(along, across)
    if side == 0:
        coords = np.stack((x, -v), axis=-1)
    elif side == 1:
        coords = np.stack((1+v, x), axis=-1)
    elif side == 2:
        coords = np.stack((1-x, 1+v), axis=-1)
    else:
        coords = np.stack((-v, 1-x), axis=-1)
    transform = cv2.getPerspectiveTransform(canonical, q.astype(np.float32))
    pixels = cv2.perspectiveTransform(coords.astype(np.float32).reshape(-1, 1, 2), transform).reshape(rows, cols, 2)
    h, w = array.shape[:2]
    if np.any(pixels[:, :, 0] < 1) or np.any(pixels[:, :, 1] < 1) or np.any(pixels[:, :, 0] > w-2) or np.any(pixels[:, :, 1] > h-2):
        return None
    return cv2.remap(array, pixels[:, :, 0], pixels[:, :, 1], cv2.INTER_LINEAR)


def periodic_border(patch, contrast):
    """Learn the visible repeat period rather than assume a checker cell size."""
    if patch is None:
        return None
    values = patch.astype(np.float32)
    if float(np.percentile(values, 90)-np.percentile(values, 10)) < max(24., .32*contrast):
        return None
    values -= values.mean(axis=1, keepdims=True)
    energy = np.mean(values*values, axis=1)
    useful = energy > max(36., (.08*contrast)**2)
    if np.count_nonzero(useful) < 3:
        return None
    a = values[useful]
    # Autocorrelation via zero-padded FFT keeps multi-scale evaluation cheap.
    n = a.shape[1]
    fft = np.fft.rfft(a, n=2*n, axis=1)
    ac = np.fft.irfft(fft*np.conj(fft), n=2*n, axis=1)[:, :n]
    squares = np.cumsum(a*a, axis=1)
    half = np.arange(2, max(3, n//10))
    def correlation(lags):
        left = squares[:, n-lags-1]
        right = squares[:, -1, None]-squares[:, lags-1]
        return ac[:, lags]/(np.sqrt(np.maximum(left*right, 0.))+1e-6)
    quality = np.median(np.minimum(-correlation(half), correlation(2*half)), axis=0)
    index = int(np.argmax(quality))
    best, period = float(quality[index]), int(half[index]*2)
    if best < .30:
        return None
    return dict(period_samples=period, periodicity=best)


class Detector:
    def __init__(self, max_width=640):
        if not 320 <= int(max_width) <= 1280:
            raise ValueError('max_width must be in [320,1280]')
        self.max_width = int(max_width)

    def _textures(self, gray):
        f = gray.astype(np.float32)
        contrast = float(np.percentile(f, 95)-np.percentile(f, 5))
        gx = cv2.Sobel(f, cv2.CV_32F, 1, 0, ksize=3)/8.
        gy = cv2.Sobel(f, cv2.CV_32F, 0, 1, ksize=3)/8.
        masks = []
        for fraction in (.010, .019, .033):
            k = max(3, int(round(min(gray.shape)*fraction)) | 1)
            mean = cv2.boxFilter(f, -1, (k, k))
            std = np.sqrt(np.maximum(0., cv2.boxFilter(f*f, -1, (k, k))-mean*mean))
            a = cv2.boxFilter(gx*gx, -1, (k, k))
            b = cv2.boxFilter(gy*gy, -1, (k, k))
            c = cv2.boxFilter(gx*gy, -1, (k, k))
            eig = .5*(a+b-np.sqrt(np.maximum(0., (a-b)**2+4*c*c)))
            for fraction_std in (.14, .21):
                mask = ((std > max(6., fraction_std*contrast)) & (eig > max(3., (.025*contrast)**2))).astype(np.uint8)*255
                masks.append(mask)
        return masks, contrast

    def _side_support(self, gray, texture, q, contrast, solid_allowed):
        sides = []
        for side in range(4):
            inner = sample_band(texture, q, side, -.13, -.035)
            if inner is None:
                return None
            inner_density = float(np.mean(inner > 100))
            best = None
            for width in (.025, .045, .075, .12, .18):
                outer = sample_band(texture, q, side, .008, width)
                if outer is None:
                    continue
                chunks = [float(np.mean(x > 100)) for x in np.array_split(outer, 8, axis=1)]
                density = float(np.mean(outer > 100))
                coverage = sum(x >= .35 for x in chunks)/8.
                if density >= .46 and coverage >= .875:
                    pattern = periodic_border(sample_band(gray, q, side, .008, width, rows=10, cols=160), contrast)
                    if pattern is None:
                        continue
                    item = dict(kind='texture', coverage=coverage, contrast=density-inner_density,
                                band_fraction=width, density=density, inner_density=inner_density, **pattern)
                    if best is None or item['contrast'] > best['contrast']:
                        best = item
            if best is None and solid_allowed:
                inside = sample_band(gray, q, side, -.08, -.018)
                for width in (.025, .045, .075, .12):
                    outside = sample_band(gray, q, side, .012, width)
                    if outside is None or inside is None:
                        continue
                    delta = np.abs(np.median(outside, axis=0)-np.median(inside, axis=0))
                    coverage = float(np.mean(delta > max(12., .14*contrast)))
                    if coverage >= .85 and float(np.std(outside)) < max(18., .22*contrast):
                        best = dict(kind='solid_ring', coverage=coverage,
                                    contrast=float(np.median(delta)/255.), band_fraction=width)
                        break
            if best is None:
                return None
            sides.append(best)
        if not solid_allowed and sum(s['contrast'] >= .16 for s in sides) < 2:
            return None
        return sides

    @staticmethod
    def _already_accepted(corners, accepted):
        # Exactly the final NMS rule. Selected candidates still undergo all
        # four-border checks; suppressed candidates cannot change the result.
        return any(polygon_iou(corners, old.corners) > .65 for old in accepted)

    def detect(self, image, debug=False):
        start = time.perf_counter()
        if image is None or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 80:
            raise ValueError('expected uint8 BGR image, at least 80 pixels per side')
        scale = min(1., self.max_width/float(image.shape[1]))
        frame = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        textures, contrast = self._textures(gray)
        pooled = np.maximum.reduce(textures)
        # Proposals from multi-scale 2-D texture and from actual nested solid
        # regions. Never construct an opening from a requested metric size.
        proposals = []
        morph = max(3, int(round(min(gray.shape)*.010)) | 1)
        kernel = np.ones((morph, morph), np.uint8)
        masks = []
        for texture in textures:
            cleaned = cv2.morphologyEx(texture, cv2.MORPH_CLOSE, kernel)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
            masks.append((cleaned, False))
        _, binary = cv2.threshold(cv2.GaussianBlur(gray, (3, 3), 0), 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        masks.extend([(binary, True), (255-binary, True)])
        for mask, solid in masks:
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if hierarchy is None:
                continue
            for i, contour in enumerate(contours):
                parent_id = int(hierarchy[0, i, 3])
                if parent_id < 0:
                    continue
                q = contour_quad(contour, gray.shape)
                if q is None:
                    continue
                # Only holes (odd hierarchy depth), not a printed rectangle or
                # the connected exterior background, are opening proposals.
                depth, ancestor = 0, parent_id
                while ancestor >= 0:
                    depth += 1
                    ancestor = int(hierarchy[0, ancestor, 3])
                if depth % 2 != 1:
                    continue
                solid_allowed = False
                if solid:
                    outer = contour_quad(contours[parent_id], gray.shape)
                    if outer is None:
                        continue
                    ratio = abs(cv2.contourArea(outer))/abs(cv2.contourArea(q))
                    if not 1.08 <= ratio <= 5.:
                        continue
                    solid_allowed = True
                if any(polygon_iou(q, p[0]) > .96 and solid_allowed == p[1] for p in proposals):
                    continue
                proposals.append((q, solid_allowed))
        found = []
        # Prefer strong/large openings; cap work to stay bounded in noisy scenes.
        proposals.sort(key=lambda p: abs(cv2.contourArea(p[0])), reverse=True)
        selected = sorted(proposals[:40], key=lambda p: abs(cv2.contourArea(p[0]/scale)), reverse=True)
        for q, solid in selected:
            if self._already_accepted(q/scale, found):
                continue
            support = self._side_support(gray, pooled, q, contrast, solid)
            if support is None:
                continue
            score = float(np.mean([.6*s['coverage']+.4*min(1., s['contrast']) for s in support]))
            method = 'multiscale_texture_ring' if all(s['kind']=='texture' for s in support) else 'nested_border'
            found.append(Detection(q/scale, quad_center(q)/scale, score, method, support))
        # Large texture kernels erode the same opening into smaller holes.
        # Among overlapping, four-side-validated candidates retain the outermost
        # observed hole, not whichever erosion has the highest contrast.
        found.sort(key=lambda d: abs(cv2.contourArea(d.corners)), reverse=True)
        unique = []
        for det in found:
            if not any(polygon_iou(det.corners, old.corners) > .65 for old in unique):
                unique.append(det)
        result = dict(target_count=len(unique), reason='one_visual_candidate' if len(unique)==1 else ('multiple_candidates' if unique else 'no_complete_frame'),
                      processing_ms=1000.*(time.perf_counter()-start), proposal_count=len(proposals),
                      candidates=[d.as_dict() for d in unique])
        if debug:
            result['texture_mask'] = pooled
            result['proposal_quads'] = [p[0]/scale for p in proposals]
        return unique, result


def annotate(image, detections, text='VISUAL ONLY / NO FLIGHT OUTPUT'):
    out = image.copy()
    for i, d in enumerate(detections):
        q = np.round(d.corners).astype(np.int32)
        cv2.polylines(out, [q], True, (0, 220, 0), 2)
        c = tuple(int(round(v)) for v in d.center)
        cv2.drawMarker(out, c, (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(out, '%d %.2f'%(i+1, d.score), tuple(q[0]), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
    cv2.drawMarker(out, (out.shape[1]//2, out.shape[0]//2), (255, 100, 0), cv2.MARKER_CROSS, 12, 1)
    cv2.rectangle(out, (0, out.shape[0]-26), (out.shape[1], out.shape[0]), (25, 25, 25), -1)
    cv2.putText(out, text[:85], (6, out.shape[0]-9), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
    return out
