import math
import numpy as np
import yaml
from lc_planner.transformations import euler_matrix

########################################################################################################################
# region Helper functions
########################################################################################################################


def intrinsics_parameters_to_matrix(fov_deg, w, h):
    """
    Args:
        fov_deg: (float) horizontal FOV angle in degrees.
        w: (int) width in pixels.
        h: (int) height in pixels.
    Returns: (np.ndarray, np.float32, shape=(3, 3))
             camara intrinsics matrix.
    """
    fov_rad = fov_deg * math.pi / 180.0
    fx = fy = (w / 2) / math.tan(fov_rad / 2)
    cx, cy = w / 2, h / 2

    return np.array([[fx, 0., cx],
                     [0., fy, cy],
                     [0., 0., 1.]], dtype=np.float32)


def intrinsics_matrix_to_parameters(matrix):
    """
    Args:
        matrix (np.ndarray, dtype=np.float32, shape=(3, 3)): intrinsics matrix
    Returns:
        fov_deg: (float) horizontal FOV angle in degrees.
        w: (float) width in pixels.
        h: (float) height in pixels.
    """
    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]
    w, h = 2 * cx, 2 * cy
    fov_rad = 2 * math.atan(w / (2 * fx))  # lies in [-pi, pi]
    if fov_rad < 0:
        fov_rad += 2 * np.pi
    fov_deg = fov_rad * 180.0 / np.pi
    return fov_deg, w, h


def get_transform_from_xyzrpy(x, y, z, roll, pitch, yaw):
    # convert roll, pitch, yaw to radians.
    # Interpretation: transformation matrix FROM the new coordinate frame rotated according to
    #                 x, y, z, roll, pitch, yaw, TO the original coordinate frame.
    # NOTE: roll, pitch, yaw correspond to rotation about the x, y, z axes respectively about the FIXED axes in the
    #       ORIGINAL frame.
    roll, pitch, yaw = roll * np.pi/180., pitch * np.pi/180., yaw * np.pi/180.
    transform = euler_matrix(roll, pitch, yaw)
    transform[:3, 3] = [x, y, z]
    return transform.astype(np.float32)

# endregion
########################################################################################################################


class LCConfig(object):
    @staticmethod
    def load_yaml_config(path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # default params: empty dictionaries
        CAMERA_PARAMS, LASER_PARAMS = {}, {}

        if 'LASER_PARAMS' in data:
            LASER_PARAMS = data['LASER_PARAMS']

            # convert 2D lists to numpy arrays
            for key in ['cam_to_laser', 'laser_to_world']:
                if (key in LASER_PARAMS) and (LASER_PARAMS[key] is not None):
                    LASER_PARAMS[key] = np.array(LASER_PARAMS[key], dtype=np.float32)

        if 'CAMERA_PARAMS' in data:
            CAMERA_PARAMS = data['CAMERA_PARAMS']

            # convert 2D lists to numpy arrays
            for key in ['matrix', 'cam_to_world']:
                if (key in CAMERA_PARAMS) and (CAMERA_PARAMS[key] is not None):
                    CAMERA_PARAMS[key] = np.array(CAMERA_PARAMS[key], dtype=np.float32)

        return CAMERA_PARAMS, LASER_PARAMS

    def __init__(self, config_file, default_config_file=None):
        self.CAMERA_PARAMS, self.LASER_PARAMS = {}, {}

        # load defaults
        if default_config_file is not None:
            self.CAMERA_PARAMS, self.LASER_PARAMS = self.load_yaml_config(default_config_file)

        CAMERA_PARAMS, LASER_PARAMS = self.load_yaml_config(config_file)
        self.CAMERA_PARAMS.update(CAMERA_PARAMS)
        self.LASER_PARAMS.update(LASER_PARAMS)

        self.postprocess()

    def postprocess(self):
        # Compute camera matrix if not already specified.
        if self.CAMERA_PARAMS['matrix'] is None:
            self.CAMERA_PARAMS['matrix'] = intrinsics_parameters_to_matrix(
                self.CAMERA_PARAMS['fov'],
                self.CAMERA_PARAMS['width'],
                self.CAMERA_PARAMS['height']
            )
        # If matrix is specified, overwrite the horizontal FOV
        else:
            fov, w, h = intrinsics_matrix_to_parameters(self.CAMERA_PARAMS['matrix'])
            self.CAMERA_PARAMS['fov'] = fov

        # Compute transforms.
        self.TRANSFORMS = {}

        if self.CAMERA_PARAMS['cam_to_world'] is not None:
            self.TRANSFORMS['cam_to_world'] = self.CAMERA_PARAMS['cam_to_world']
        else:
            self.TRANSFORMS['cam_to_world'] = get_transform_from_xyzrpy(self.CAMERA_PARAMS['x'],
                                                                        self.CAMERA_PARAMS['y'],
                                                                        self.CAMERA_PARAMS['z'],
                                                                        self.CAMERA_PARAMS['roll'],
                                                                        self.CAMERA_PARAMS['pitch'],
                                                                        self.CAMERA_PARAMS['yaw'])
        self.TRANSFORMS['world_to_cam'] = np.linalg.inv(self.TRANSFORMS['cam_to_world'])

        if self.LASER_PARAMS['laser_to_world'] is not None:
            self.TRANSFORMS['laser_to_world'] = self.LASER_PARAMS['laser_to_world']
        else:
            self.TRANSFORMS['laser_to_world'] = get_transform_from_xyzrpy(self.LASER_PARAMS['x'],
                                                                          self.LASER_PARAMS['y'],
                                                                          self.LASER_PARAMS['z'],
                                                                          self.LASER_PARAMS['roll'],
                                                                          self.LASER_PARAMS['pitch'],
                                                                          self.LASER_PARAMS['yaw'])
        self.TRANSFORMS['world_to_laser'] = np.linalg.inv(self.TRANSFORMS['laser_to_world'])

        if self.LASER_PARAMS['cam_to_laser'] is not None:
            self.TRANSFORMS['cam_to_laser'] = self.LASER_PARAMS['cam_to_laser']
        else:
            self.TRANSFORMS['cam_to_laser'] = self.TRANSFORMS['world_to_laser'].dot(self.TRANSFORMS['cam_to_world'])
        self.TRANSFORMS['laser_to_cam'] = np.linalg.inv(self.TRANSFORMS['cam_to_laser'])
