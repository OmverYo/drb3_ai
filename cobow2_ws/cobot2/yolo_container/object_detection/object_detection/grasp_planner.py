from dataclasses import dataclass, field
import json
import math
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import SAM
import cv2
import numpy as np

# SAM 마스크 추출

class SamMaskModel:
    def __init__(self, package_name='object_detection'):

        share = get_package_share_directory(package_name)
        self.model_path = os.getenv('SAM_MODEL_PATH', os.path.join(share, 'resource', 'sam2.1_s.pt'))
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError('Set SAM_MODEL_PATH to sam2.1_s.pt: ' + self.model_path)
        self.model = SAM(self.model_path)
        self.device = os.getenv('SAM_DEVICE', '0')

    def segment_boxes(self, frame, detections):
        if not detections:
            return []
        results = self.model.predict(source=frame,
            bboxes=np.array([d['box'] for d in detections], dtype=np.float32),
            device=self.device, verbose=False)
        if not results or results[0].masks is None:
            raise ValueError('SAM returned no masks')
        masks = results[0].masks.data.cpu().numpy() >= 0.5
        # Never resize letterboxed output as though padding were image content,
        # or silently associate incomplete batch results with the wrong instance.
        if len(masks) != len(detections) or masks.shape[1:] != frame.shape[:2]:
            raise ValueError('SAM output count/shape mismatch; check Ultralytics version')
        for mask, detection in zip(masks, detections):
            yy, xx = np.nonzero(mask)
            if len(xx) < 20:
                raise ValueError('Empty or tiny SAM mask')
            x0, y0, x1, y1 = detection['box']
            inside = (xx >= x0 - 3) & (xx <= x1 + 3) & (yy >= y0 - 3) & (yy <= y1 + 3)
            if inside.mean() < 0.9:
                raise ValueError('SAM mask spills outside its YOLO prompt; reacquire')
        return list(masks)

# 평면 충돌 검사 및 회전각 탐색

@dataclass
class PlanarPlan:
    yaw_deg: float
    opening_mm: float
    target_width_mm: float
    clearance_mm: float
    finger_polygons: list

class PlanarSafetyEvaluator:
    def __init__(self, scale=2.0, max_opening_mm=100.0,
                 finger_radial_mm=12.0, finger_tangent_mm=4.0,
                 opening_margin_mm=1.0, obstacle_margin_mm=1.0,
                 fixed_opening_mm=48.0):
        self.scale = float(scale)
        self.max_opening_mm = float(max_opening_mm)
        self.radial = float(finger_radial_mm)
        self.tangent = float(finger_tangent_mm)
        self.opening_margin = float(opening_margin_mm)
        self.obstacle_margin = float(obstacle_margin_mm)
        self.fixed_opening = float(fixed_opening_mm)
        if self.scale <= 0:
            raise ValueError('GRASP_MAP_PX_PER_MM must be positive')
        if not 0 < self.fixed_opening <= self.max_opening_mm:
            raise ValueError('RG2_FIXED_OPENING_MM must be within the gripper range')

    def prepare(self, target, obstacles, observed, center_px=None):
        target = np.asarray(target, dtype=bool)
        obstacles = np.asarray(obstacles, dtype=bool)
        observed = np.asarray(observed, dtype=bool)
        if target.shape != obstacles.shape or target.shape != observed.shape:
            raise ValueError('Metric mask shapes differ')
        if np.count_nonzero(target) < 10:
            raise ValueError('Target metric mask is empty')
        if np.any(target & obstacles):
            raise ValueError('Target and neighbor masks overlap; segmentation is ambiguous')
        yy, xx = np.nonzero(target)
        if center_px is None:
            center_px = np.array([xx.mean(), yy.mean()], dtype=float)
        else:
            center_px = np.asarray(center_px, dtype=float)
            if center_px.shape != (2,) or not np.all(np.isfinite(center_px)):
                raise ValueError('Invalid grasp centre')
        points = (np.column_stack((xx, yy)) - center_px) / self.scale
        # Outside the camera view and missing depth are unknown, never free.
        occupied = (obstacles | ~observed).astype(np.uint8)
        occupied[[0, -1], :] = 1
        occupied[:, [0, -1]] = 1
        distance = cv2.distanceTransform(1 - occupied, cv2.DIST_L2, 5) / self.scale
        return dict(target=target, center_px=center_px, points=points,
                    occupied=occupied, distance=distance)

    def _polygon(self, center_px, direction, tangent, low, high):
        return np.rint(np.array([
            center_px + self.scale * (radial * direction + side * self.tangent / 2 * tangent)
            for radial, side in [(low, -1), (low, 1), (high, 1), (high, -1)]
        ])).astype(np.int32)

    def evaluate(self, prepared, yaw_deg):
        theta = math.radians(float(yaw_deg))
        direction = np.array([math.cos(theta), math.sin(theta)])
        tangent = np.array([-direction[1], direction[0]])
        projection = prepared['points'] @ direction
        lo, hi = float(projection.min()), float(projection.max())
        # Full extents, NOT percentiles. A fixed centre may be asymmetric;
        # twice the larger extent prevents a finger from landing on the piece.
        pixel_guard = math.sqrt(2) / self.scale
        contact_half = max(abs(lo), abs(hi)) + pixel_guard
        required_opening = 2 * (contact_half + self.opening_margin)
        opening = self.fixed_opening
        if required_opening > opening:
            return None
        # Include inward closing travel down to the smaller contact extent.
        close_half = max(0.0, min(abs(lo), abs(hi)) - pixel_guard)
        sweep = np.zeros_like(prepared['target'], dtype=np.uint8)
        opened = np.zeros_like(sweep)
        polygons = []
        h, w = sweep.shape
        for sign in (-1, 1):
            d = sign * direction
            polygon = self._polygon(prepared['center_px'], d, tangent,
                                    opening / 2, opening / 2 + self.radial)
            swept = self._polygon(prepared['center_px'], d, tangent,
                                  close_half, opening / 2 + self.radial)
            if (np.any(swept[:, 0] < 0) or np.any(swept[:, 0] >= w)
                    or np.any(swept[:, 1] < 0) or np.any(swept[:, 1] >= h)):
                return None
            cv2.fillConvexPoly(sweep, swept, 1)
            cv2.fillConvexPoly(opened, polygon, 1)
            polygons.append(polygon)
        if np.any(opened.astype(bool) & prepared['target']):
            return None
        clearance = float(prepared['distance'][sweep.astype(bool)].min())
        if clearance <= self.obstacle_margin + pixel_guard:
            return None
        return PlanarPlan(float(yaw_deg) % 180, opening, hi - lo,
                          clearance, polygons)

    def search(self, prepared):
        # Evaluate every integer degree: coarse-only searches can miss a narrow gap.
        best = None
        for yaw in range(180):
            plan = self.evaluate(prepared, yaw)
            if plan is not None and (best is None or
                (plan.clearance_mm, -plan.opening_mm) >
                (best.clearance_mm, -best.opening_mm)):
                best = plan
        return best

# SAM·깊이 영상 기반 파지 계획

@dataclass
class GraspPlanResult:
    success: bool = False
    planner: str = 'sam_yaw_search'
    camera_position_mm: list = field(default_factory=lambda: [0., 0., 0.])
    safe_yaw_deg: float = 0.
    grasp_width_mm: float = 0.
    target_width_mm: float = 0.
    clearance_mm: float = 0.
    score: float = 0.
    yaw_reference: str = 'camera_local'
    closing_axis_camera: list = field(default_factory=lambda: [0., 0., 0.])
    approach_axis_camera: list = field(default_factory=lambda: [0., 0., 0.])
    message: str = ''

class SamGraspPlanner:
    def __init__(self, logger=None):
        self.logger = logger

    @staticmethod
    def geometry():
        return PlanarSafetyEvaluator(
            scale=float(os.getenv('GRASP_MAP_PX_PER_MM', '2')),
            max_opening_mm=float(os.getenv('RG2_MAX_OPENING_MM', '100')),
            finger_radial_mm=float(os.getenv('RG2_FINGER_RADIAL_MM', '12')),
            finger_tangent_mm=float(os.getenv('RG2_FINGER_TANGENT_MM', '4')),
            opening_margin_mm=float(os.getenv('RG2_OPENING_MARGIN_MM', '1')),
            obstacle_margin_mm=float(os.getenv('RG2_OBSTACLE_MARGIN_MM', '1')),
            fixed_opening_mm=float(os.getenv('RG2_FIXED_OPENING_MM', '48')),
        )

    @staticmethod
    def axes(approach_camera=None):
        # No unvalidated PCA of a partial board/table image. The normal is a
        # calibration input expressed in camera optical coordinates, toward board.
        n = np.array(approach_camera if approach_camera is not None else
                     json.loads(os.getenv('GRASP_APPROACH_CAMERA', '[0,0,1]')), dtype=float, copy=True)
        if n.shape != (3,) or not np.all(np.isfinite(n)) or np.linalg.norm(n) < 1e-6:
            raise ValueError('Invalid GRASP_APPROACH_CAMERA')
        n /= np.linalg.norm(n)
        if n[2] < 0.5:
            raise ValueError('Approach must point from camera toward the board')
        bx = np.array([1., 0., 0.]) - n[0] * n
        bx /= np.linalg.norm(bx)
        by = np.cross(n, bx)
        return bx, by, n

    @staticmethod
    def _mask_plane_distance(mask, z_mm, valid, rays, normal):
        core = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        count = np.count_nonzero(core)
        usable = core & valid
        if count < 20 or np.count_nonzero(usable) < max(20, 0.7 * count):
            raise ValueError('Insufficient valid depth inside a SAM mask')
        distances = z_mm[usable] * (rays[usable] @ normal)
        distance = float(np.median(distances))
        if np.percentile(np.abs(distances - distance), 90) > 5.0:
            raise ValueError('Mask depth is inconsistent with a flat piece; reacquire RGB-D')
        return distance

    @staticmethod
    def select_target_by_base_xy(detections, masks, depth, depth_scale, intrinsics,
                                 base_from_camera, reference_xy, target_name=None,
                                 min_score=0.5, max_distance_mm=15., ambiguity_mm=5.):
        """Same-frame SAM centres, matched to a physical Base XY, never image centre."""
        T = np.asarray(base_from_camera, dtype=float).reshape(4, 4)
        ref = np.asarray(reference_xy, dtype=float)
        R = T[:3, :3]
        if (not np.all(np.isfinite(T)) or not np.allclose(T[3], [0,0,0,1]) or
                not np.allclose(R.T @ R, np.eye(3), atol=1e-4) or
                not np.isclose(np.linalg.det(R), 1., atol=1e-4)):
            raise ValueError('base_from_camera must be a rigid transform in millimetres')
        if ref.shape != (2,) or not np.all(np.isfinite(ref)):
            raise ValueError('Invalid reference Base XY')
        if not (0 < max_distance_mm <= 20 and 0 < ambiguity_mm < max_distance_mm):
            raise ValueError('Target gates require 0 < ambiguity < distance <= 20 mm')
        if len(detections) != len(masks):
            raise ValueError('Detection/mask count mismatch')
        fx,fy,cx,cy = (float(intrinsics[k]) for k in ('fx','fy','ppx','ppy'))
        if min(fx,fy) <= 0 or not np.all(np.isfinite([fx,fy,cx,cy])):
            raise ValueError('Invalid intrinsics')
        normal = R.T @ np.array([0.,0.,-1.])
        SamGraspPlanner.axes(normal)
        z = np.asarray(depth, float) * float(depth_scale) * 1000
        valid = np.isfinite(z) & (z > 50) & (z < 2000)
        yy,xx = np.indices(z.shape)
        rays = np.stack(((xx-cx)/fx,(yy-cy)/fy,np.ones_like(z)),axis=-1)
        candidates = []
        for index,(detection,mask) in enumerate(zip(detections,masks)):
            # Match geometry before class/score filtering: a wrong nearby piece
            # must not become eligible because the true target scored lower.
            if mask is None or np.shape(mask) != z.shape:
                raise ValueError('Invalid instance mask')
            mask = np.asarray(mask,bool)
            distance = SamGraspPlanner._mask_plane_distance(mask,z,valid,rays,normal)
            cloud = rays[mask] * (distance / (rays[mask] @ normal))[:,None]
            center = cloud.mean(axis=0)
            base = R @ center + T[:3,3]
            error = float(np.linalg.norm(base[:2]-ref))
            candidates.append((error,index))
        candidates.sort()
        if not candidates or candidates[0][0] > max_distance_mm:
            raise ValueError('No piece within the requested Base XY distance gate')
        if len(candidates)>1 and candidates[1][0]-candidates[0][0] < ambiguity_mm:
            raise ValueError('Ambiguous target: two pieces near the reference Base XY')
        distance,index = candidates[0]
        selected = detections[index]
        if target_name not in (None,'','*') and selected['name'] != target_name:
            raise ValueError('Nearest piece class differs from the voice target')
        if selected['score'] < min_score:
            raise ValueError('Nearest piece confidence is below target threshold')
        return index, distance, normal

    def plan(self, depth, depth_scale, intrinsics, masks, target_index, approach_camera=None,
             base_from_camera=None, reference_base_xy_mm=None):
        result = GraspPlanResult()
        try:
            geometry = self.geometry()
            if not 0 <= target_index < len(masks) or any(m is None for m in masks):
                raise ValueError('Every detected piece needs a valid SAM mask')
            if any(m.shape != depth.shape for m in masks):
                raise ValueError('RGB/SAM/depth dimensions differ')
            target = np.asarray(masks[target_index], bool)
            if (np.any(target[:3]) or np.any(target[-3:]) or
                    np.any(target[:, :3]) or np.any(target[:, -3:])):
                raise ValueError('Target touches image boundary; use a higher inspection pose')
            bx, by, normal = self.axes(approach_camera)
            z = depth.astype(float) * float(depth_scale) * 1000
            valid = np.isfinite(z) & (z > 50) & (z < 2000)
            fx, fy, cx, cy = (float(intrinsics[k]) for k in ('fx', 'fy', 'ppx', 'ppy'))
            if not np.all(np.isfinite([fx, fy, cx, cy])) or min(fx, fy) <= 0:
                raise ValueError('Invalid camera intrinsics')
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
            invK = np.linalg.inv(K)
            yy, xx = np.indices(depth.shape)
            rays = np.stack(((xx - cx) / fx, (yy - cy) / fy, np.ones_like(z)), axis=-1)
            d = self._mask_plane_distance(target, z, valid, rays, normal)
            target_xyz = rays[target] * (d / (rays[target] @ normal))[:, None]
            origin = target_xyz.mean(axis=0)
            # 검사 중심을 실제 하강 XY에 맞춘다. Z는 말의 측정 평면을 사용한다.
            if reference_base_xy_mm is not None:
                T = np.asarray(base_from_camera, dtype=float).reshape(4, 4)
                base = T[:3, :3] @ origin + T[:3, 3]
                base[:2] = np.asarray(reference_base_xy_mm, dtype=float)
                origin = T[:3, :3].T @ (base - T[:3, 3])
            radius = float(os.getenv('LOCAL_GRASP_RADIUS_MM', '100'))
            if not np.isfinite(radius) or radius < geometry.max_opening_mm / 2 + geometry.radial + 5 or radius > 300:
                raise ValueError('LOCAL_GRASP_RADIUS_MM must fit the gripper and be <= 300')
            size = int(np.ceil(2 * radius * geometry.scale)) + 1
            nk = normal @ invK

            def homography(distance):
                H = np.stack((distance * (bx @ invK) - (bx @ origin) * nk,
                              distance * (by @ invK) - (by @ origin) * nk, nk))
                return np.array([[geometry.scale, 0, radius * geometry.scale],
                                 [0, geometry.scale, radius * geometry.scale], [0, 0, 1.]]) @ H

            def warp(mask, distance):
                return cv2.warpPerspective(mask.astype(np.uint8), homography(distance),
                                           (size, size), flags=cv2.INTER_NEAREST).astype(bool)

            observed = warp(valid, d)
            # Do not allow any finger to extend beyond the measured camera footprint.
            observed = cv2.erode(observed.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
            target_metric = warp(target, d)
            obstacles = np.zeros((size, size), bool)
            for i, mask in enumerate(masks):
                if i == target_index:
                    continue
                distance = self._mask_plane_distance(mask, z, valid, rays, normal)
                obstacles |= warp(mask, distance)
            raster_center = np.array([radius * geometry.scale] * 2, dtype=float)
            prepared = geometry.prepare(
                target_metric, obstacles, observed, center_px=raster_center
            )
            plan = geometry.search(prepared)
            if plan is None:
                raise ValueError('No visible collision-free direction in 0..179 degrees')
            # Use EXACTLY the centre tested by the raster planner as the output centre.
            local_center = prepared['center_px'] / geometry.scale - radius
            center_camera = origin + local_center[0] * bx + local_center[1] * by
            angle = np.deg2rad(plan.yaw_deg)
            result.success = True
            result.camera_position_mm = center_camera.tolist()
            result.safe_yaw_deg = plan.yaw_deg
            result.target_width_mm = plan.target_width_mm
            result.grasp_width_mm = plan.opening_mm
            result.clearance_mm = plan.clearance_mm
            result.closing_axis_camera = (np.cos(angle) * bx + np.sin(angle) * by).tolist()
            result.approach_axis_camera = normal.tolist()
            result.message = 'Planar finger descent/closure footprint passed; rotate above obstacles first'
        except Exception as error:
            result.message = str(error)
            if self.logger:
                self.logger.warning('Grasp plan rejected: ' + result.message)
        return result