import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

build_dir = os.path.dirname(os.path.abspath(__file__)) + '/../../../build'
sys.path.append(build_dir)  # should contain a compiled planner_py.so file
import planner_py

from lc_planner.config import LCConfig


class Planner(object):
    def __init__(self, config, debug=False):
        self._debug = debug
        self._planner = None

        # camera parameters
        self.cparams = planner_py.CameraParameters()
        self.cparams.width = config.CAMERA_PARAMS['width']
        self.cparams.height = config.CAMERA_PARAMS['height']
        self.cparams.fps = config.CAMERA_PARAMS['fps']
        self.cparams.cam_matrix = config.CAMERA_PARAMS['matrix'].T.ravel()
        self.cparams.cam_to_laser = config.TRANSFORMS['cam_to_laser'].T.ravel()

        # laser parameters
        self.lparams = planner_py.LaserParameters()
        self.lparams.fov = config.LASER_PARAMS['fov']
        self.lparams.thickness = config.LASER_PARAMS['thickness']
        self.lparams.divergence = config.LASER_PARAMS['divergence']
        self.lparams.max_omega = config.LASER_PARAMS['max_omega']
        self.lparams.max_alpha = config.LASER_PARAMS['max_alpha']

    def confidence2entropy(self, confidence_map):
        """
        Args:
            confidence_map: (np.ndarray, dtype=float32, shape=(X, Z, 3+)) confidence map of detector.
                            Axis 0 corresponds to increasing X (camera frame) / decreasing Y (velo frame).
                            Axis 1 corresponds to increasing Z (camera frame) / increasing X (velo frame).
                            Axis 2 corresponds to (x, z, c):
                                - x  : x in camera frame.
                                - z  : z in camera frame.
                                - c+ : confidence score of various factors, lying in [0, 1].
        Returns:
            entropy_map: (np.ndarray, dtype=float32, shape=(X, Z, 3)) entropy map of detector.
                             Axis 0 corresponds to increasing X (camera frame) / decreasing Y (velo frame).
                             Axis 1 corresponds to increasing Z (camera frame) / increasing X (velo frame).
                             Axis 2 corresponds to (x, z, c):
                                 - x : x in camera frame.
                                 - z : z in camera frame.
                                 - e : entopy.
        """
        xz = confidence_map[:, :, :2]  # (X, Z, 2)
        p  = confidence_map[:, :, 2:]  # (X, Z, K)
        e  = (-p * np.log(1e-5 + p)).mean(axis=2, keepdims=True)  # (X, Z, 1)
        entropy_map = np.concatenate((xz, e), axis=2)  # (X, Z, 3)
        return entropy_map

    def _visualize_layout(self, map, show=True):
        """
        Args:
            map: (np.ndarray, dtype=float32, shape=(X, Y)) map of the objective function.
        """
        layout = self._planner.getLayoutForVis()
        layout = np.array(layout)  # (RAYS, RANGES_PER_RAY)
        locs = layout.ravel()  # (RAYS, RANGES_PER_RAY)

        x = np.array([loc.x for loc in locs])
        z = np.array([loc.z for loc in locs])
        r = np.array([loc.r for loc in locs])
        tcam = np.array([loc.theta_cam for loc in locs])
        tlas = np.array([loc.theta_las for loc in locs])
        ki = np.array([loc.ki for loc in locs])
        kj = np.array([loc.kj for loc in locs])

        u = []
        for ki_, kj_ in zip(ki, kj):
            if ki_ == -1 or kj_ == -1:
                u_ = 0
            else:
                u_ = map[ki_, kj_]
            u.append(u_)

        plt.scatter(x, z, c=u, cmap='hot', s=1)
        if show:
            plt.title("PLANNER: Camera Ray Points w/ Interpolated Reward", fontsize='x-large')
            plt.show()

    @staticmethod
    def _visualize_curtain_xy(umap, design_points):
        """
        Args:
            umap: (np.ndarray, dtype=float32, shape=(X, Z, 3+)) confidence map of detector.
                  Axis 0 corresponds to increasing X (camera frame) / decreasing Y (velo frame).
                  Axis 1 corresponds to increasing Z (camera frame) / increasing X (velo frame).
                  Axis 2 corresponds to (x, z, c):
                    - x : x in camera frame.
                    - z : z in camera frame.
                    - u : uncertainty score lying in [0, 1].
        """
        flattened_umap = umap.reshape(-1, 3)
        x, z, u = flattened_umap[:, 0], flattened_umap[:, 1], flattened_umap[:, 2]
        plt.scatter(x, z, c=u, cmap='hot')
        plt.plot(design_points[:, 0], design_points[:, 1], linewidth=1, c='b')
        plt.scatter(design_points[:, 0], design_points[:, 1], s=1, c='w')
        plt.show()

    def _visualize_curtain_rt(self, map, design_points, title=None, show=True):
        """
        Args:
            map: (np.ndarray, dtype=float32, shape=(X, Y)) map of the objective function.
        """
        self._visualize_layout(map, show=False)
        plt.plot(design_points[:, 0], design_points[:, 1], linewidth=1, c='b')
        plt.scatter(design_points[:, 0], design_points[:, 1], s=1, c='w')
        if title is not None:
            plt.title(title, fontsize='x-large')
        if show:
            plt.show()

    def get_design_points(self, confidence_map):
        pass


class PlannerXY(Planner):
    def __init__(self, config, version=2, pts_per_cam_ray=80, debug=False):
        super(PlannerXY, self).__init__(config, debug)
        self._planner_class = planner_py.PlannerV1 if version == 1 else planner_py.PlannerV2
        self._pts_per_cam_ray = pts_per_cam_ray

    def get_design_points(self, confidence_map):
        """
        Args:
            confidence_map: (np.ndarray, dtype=float32, shape=(X, Z, 3+)) confidence map of detector.
                            Axis 0 corresponds to increasing X (camera frame) / decreasing Y (velo frame).
                            Axis 1 corresponds to increasing Z (camera frame) / increasing X (velo frame).
                            Axis 2 corresponds to (x, z, c+):
                                - x  : x in camera frame.
                                - z  : z in camera frame.
                                - c+ : confidence score of various factors, lying in [0, 1].
        Returns:
            design_points: (np.ndarray, dtype=float32, shape=(N, 2)) point cloud of design points.
                        Each point (axis 1) contains (x, z) location of design point in camera frame.

        """
        uncertainty_map = self.confidence2entropy(confidence_map)  # (X, Z, 3)

        if not self._planner:
            if self._debug:
                im = np.flip(uncertainty_map[:, :, 2].T, axis=0)
                plt.imshow(im, cmap='hot')
                plt.title("Uncertainty")
                plt.show()

            # Create interpolator.
            umap_h, umap_w = uncertainty_map.shape[:2]
            x_min, x_max = uncertainty_map[:, :, 0].min(), uncertainty_map[:, :, 0].max()
            z_min, z_max = uncertainty_map[:, :, 1].min(), uncertainty_map[:, :, 1].max()
            self._interpolator = planner_py.CartesianNNInterpolator(umap_w, umap_h, x_min, x_max, z_min, z_max)

            # Create ranges.
            # ranges = list(invlinspace(3.0, z_max, self._pts_per_cam_ray))
            ranges = list(np.linspace(3.0, z_max, self._pts_per_cam_ray))

            self._planner = self._planner_class(self.cparams, self.lparams, ranges, self._interpolator, self._debug)

            if self._debug:
                self._visualize_layout(uncertainty_map[:, :, 2])

        design_points = self._planner.optGlobalCostDiscrete(uncertainty_map[:, :, 2])
        design_points = np.array(design_points)

        if self._debug:
            self._visualize_curtain_xy(uncertainty_map, design_points)

        return design_points


class PlannerRT(Planner):
    def __init__(self, config, ranges, num_camera_angles, version=2, debug=False):
        super(PlannerRT, self).__init__(config, debug)

        self.ranges = ranges
        self.num_camera_angles = num_camera_angles

        self._interpolator = planner_py.PolarIdentityInterpolator(self.num_camera_angles, len(self.ranges))

        planner_class = planner_py.PlannerV1 if version == 1 else planner_py.PlannerV2
        self._planner = planner_class(self.cparams, self.lparams, self.ranges, self._interpolator, self._debug)
        self.thetas = np.array(self._planner.getThetas(), dtype=np.float32)

    def draw_boundary(self):
        fig, ax = plt.subplots()
        max_range = np.max(self.ranges)

        theta_l, theta_r = self.thetas.min(), self.thetas.max()
        xl = max_range * np.sin(np.deg2rad(theta_l))
        zl = max_range * np.cos(np.deg2rad(theta_l))
        xr = max_range * np.sin(np.deg2rad(theta_r))
        zr = max_range * np.cos(np.deg2rad(theta_r))

        ax.plot([0, xl], [0, zl], c='r', linewidth=2)
        ax.plot([0, xr], [0, zr], c='r', linewidth=2)
        arc = Arc((0, 0), 2 * max_range, 2 * max_range,
                  theta1=90-theta_r, theta2=90-theta_l,
                  color='r', linewidth=2)
        ax.add_patch(arc)
        plt.xlim(xl - (xr - xl) * 0.05, xr + (xr - xl) * 0.05)
        plt.ylim(-max_range * 0.05, 1.05 * max_range)

    def get_design_points(self, umap):
        """
        Args:
            umap: (np.ndarray, dtype=float32, shape=(R, C)) objective to maximize.
                   - R is the number of ranges.
                   - C is the number of camera rays.
        Returns:
            design_points: (np.ndarray, dtype=float32, shape=(N, 2)) point cloud of design points.
                           Each point (axis 1) contains (x, z) location of design point in camera frame.

        """
        if self._debug:
            self._visualize_layout(umap)

        design_points = self._planner.optGlobalCostDiscrete(umap)
        design_points = np.array(design_points)

        if self._debug:
            self._visualize_curtain_rt(umap, design_points)

        return design_points


def test_xy(version):  # Parameters used for Virtual KITTI.
    kitti_config = LCConfig(config_file='lc_planner/test_xy.yaml', default_config_file='lc_planner/defaults.yaml')
    planner = PlannerXY(kitti_config, version=version, debug=True)

    cmap = np.load("example/confidence_map.npy")
    planner.get_design_points(cmap)


def test_rt(version):  # Parameters used by Raaj.
    raaj_config = LCConfig(config_file='lc_planner/test_rt.yaml', default_config_file='lc_planner/defaults.yaml')
    dpv = np.load("example/dpv.npy", allow_pickle=True, encoding='bytes')[()]
    ranges = dpv[b'r_candi']; assert ranges.shape == (128,)  # (128,)
    umap = dpv[b'field'].numpy(); assert umap.shape == (128, 400)  # (128, 400)
    cam_w, cam_h = dpv[b'size']

    planner = PlannerRT(raaj_config, ranges, cam_w, version=version, debug=True)
    planner.get_design_points(umap)

    for r_sampling in ["uniform", "linear"]:
        for i in range(3):
            design_points = planner._planner.randomCurtainDiscrete(r_sampling)
            design_points = np.array(design_points)[:, :2]
            planner.draw_boundary()
            planner._visualize_curtain_rt(umap, design_points, title=f"random_{r_sampling}")

    # visibility for polygon
    p1 = np.array([-10, 22], dtype=np.float32)
    p2 = np.array([ -1, 19], dtype=np.float32)
    p3 = np.array([  3, 24], dtype=np.float32)
    p4 = np.array([  0, 30], dtype=np.float32)

    surface = np.vstack([np.hstack([p1, p2]),
                         np.hstack([p2, p3]),
                         np.hstack([p3, p4]),
                         np.hstack([p4, p1])]).astype(np.float32)
    planner._visualize_layout(umap, show=False)
    for seg in surface:
        p1, p2 = seg[:2], seg[2:]
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], c='b', linewidth=3)

    planner._planner.loadLineSurface(surface)
    planner._planner.computeVisibleRanges()
    visible_pts = planner._planner.visiblePoints()
    visible_pts = np.array(visible_pts)
    if len(visible_pts) > 0:
        plt.scatter(visible_pts[:, 0], visible_pts[:, 1], c='r')
    plt.show()

    planner._planner.computeLayoutIntensities()
    layout_intensities = planner._planner.layoutIntensities()
    layout_intensities = np.array(layout_intensities)  # (RAYS, RANGES_PER_RAY)
    assert not np.isnan(layout_intensities).any()

    layout = planner._planner.getLayoutForVis()
    layout = np.array(layout)  # (RAYS, RANGES_PER_RAY)
    locs = layout.ravel()  # (RAYS * RANGES_PER_RAY,)

    x = np.array([loc.x for loc in locs])
    z = np.array([loc.z for loc in locs])
    i = layout_intensities.ravel()

    # sort according to increasing intensities
    inds = np.argsort(i)
    x, z, i = x[inds], z[inds], i[inds]

    plt.scatter(visible_pts[:, 0], visible_pts[:, 1], c='r', s=3)
    plt.scatter(x, z, c=i, cmap='viridis', s=1, vmin=0.0, vmax=1.0)
    plt.title("Intensities for polygon", fontsize='x-large')
    plt.show()

    threshold = 0.8
    for r_sampling in ["uniform", "linear"]:
        hit_prob = planner._planner.randomCurtainHitProb(threshold, r_sampling)
        print(f"Hit prob with [{r_sampling} sampling] and [threshold={threshold}]: {hit_prob}")


if __name__ == '__main__':
    test_xy(version=1)
    test_rt(version=1)
    test_xy(version=2)
    test_rt(version=2)
