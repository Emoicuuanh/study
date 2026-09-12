import cv2
import numpy as np

# Dictionary lookup table for ArUco dictionary names
ARUCO_DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def get_aruco_dictionary(dict_name: str):
    """Retrieve OpenCV ArUco dictionary by string name."""
    dict_id = ARUCO_DICTIONARIES.get(dict_name.upper(), cv2.aruco.DICT_5X5_100)
    return cv2.aruco.getPredefinedDictionary(dict_id)


def detect_aruco_markers(image: np.ndarray, dictionary):
    """
    Detect ArUco markers in a BGR/Gray image.
    Compatible with OpenCV 4.7+ (ArucoDetector) and legacy versions.
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    detector_params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector_params)

    return corners, ids


def estimate_marker_pose(corners_single: np.ndarray, marker_size: float, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
    """
    Estimate 3D Pose (rvec, tvec) of a single marker using cv2.solvePnP (IPPE_SQUARE).
    """
    half_s = marker_size / 2.0
    # 3D object points in marker coordinate frame (Z=0)
    obj_points = np.array([
        [-half_s,  half_s, 0.0],  # Top-left
        [ half_s,  half_s, 0.0],  # Top-right
        [ half_s, -half_s, 0.0],  # Bottom-right
        [-half_s, -half_s, 0.0]   # Bottom-left
    ], dtype=np.float32)

    image_points = corners_single.reshape((4, 2)).astype(np.float32)

    # Use IPPE_SQUARE for planar square markers if available
    solve_flag = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE

    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=solve_flag
    )

    return success, rvec, tvec


def rotation_matrix_to_quaternion(R: np.ndarray):
    """
    Convert a 3x3 rotation matrix to quaternion (qx, qy, qz, qw).
    """
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 1e-6:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    return qx, qy, qz, qw


def draw_aruco_debug(image: np.ndarray, corners, ids, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, rvecs: list, tvecs: list, marker_size: float):
    """
    Draw 2D marker outlines, IDs, and 3D coordinate axes on copy of the image.
    """
    output_img = image.copy()
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(output_img, corners, ids)

        if camera_matrix is not None and dist_coeffs is not None:
            axis_length = marker_size * 0.75
            for rvec, tvec in zip(rvecs, tvecs):
                if hasattr(cv2, "drawFrameAxes"):
                    cv2.drawFrameAxes(output_img, camera_matrix, dist_coeffs, rvec, tvec, axis_length)
                elif hasattr(cv2.aruco, "drawAxis"):
                    cv2.aruco.drawAxis(output_img, camera_matrix, dist_coeffs, rvec, tvec, axis_length)

    return output_img
