

from __future__ import annotations

from pathlib import Path
import sys

import cv2
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera
from config import STREAM_HEIGHT, STREAM_WIDTH
import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ===================== USER SETTINGS =====================
# Marker IDs in clockwise order starting from Top-Left:
#   [TL_id, TR_id, BR_id, BL_id]
MARKER_IDS_CW: list[int] = [0, 1, 3, 2]

# ArUco dictionary that matches your printed markers.
# Common choices: DICT_ARUCO_ORIGINAL, DICT_4X4_50, DICT_6X6_250
ARUCO_DICT_TYPE: int = cv2.aruco.DICT_ARUCO_ORIGINAL

# Physical camera parameters – used for the 3D visualisation only.
CAMERA_DIAMETER_MM: float = 17.0   # outer barrel diameter of each lens
CAMERA_DEPTH_MM: float    = 10.0   # barrel depth for cylinder rendering
# =========================================================


_ROLE_NAMES = ["TL", "TR", "BR", "BL"]
ROLE_BY_ID: dict[int, str]  = {mid: name for mid, name in zip(MARKER_IDS_CW, _ROLE_NAMES)}

# OpenCV BGR colours per corner role.
_BGR_BY_ROLE: dict[str, tuple[int, int, int]] = {
    "TL": (0,   255,   0),   # green
    "TR": (255,   0,   0),   # blue  (BGR)
    "BR": (0,     0, 255),   # red
    "BL": (0,   255, 255),   # yellow
}
# Matplotlib RGB colours (0-1).
_RGB_BY_ROLE: dict[str, tuple[float, float, float]] = {
    role: (r / 255.0, g / 255.0, b / 255.0)
    for role, (b, g, r) in _BGR_BY_ROLE.items()
}

WINDOW_NAME = "ArUco Marker Triangulation Test"


# ---------------------------------------------------------------------------
# ArUco detection (OpenCV 4.7+ ArucoDetector API)
# ---------------------------------------------------------------------------

def _build_detector(dict_type: int) -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    params     = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, params)


def _detect(
    detector:   cv2.aruco.ArucoDetector,
    image_bgr:  np.ndarray,
) -> dict[int, np.ndarray]:
    """Return {marker_id: centre_xy (float32)} for every detected marker."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    result: dict[int, np.ndarray] = {}
    if ids is None:
        return result
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        # corners[i] is shape (1, 4, 2); centre = mean of the 4 corners.
        centre = marker_corners.reshape(4, 2).mean(axis=0)
        result[int(marker_id)] = centre.astype(np.float32)
    return result


# ---------------------------------------------------------------------------
# Triangulation helpers – identical pattern to other live scripts
# ---------------------------------------------------------------------------

def _undistort_pixel(
    pt_xy: np.ndarray,
    k: np.ndarray,
    d: np.ndarray,
) -> np.ndarray:
    pt  = np.asarray(pt_xy, dtype=np.float32).reshape(1, 1, 2)
    und = cv2.undistortPoints(pt, k, d, P=k)
    return und.reshape(2).astype(np.float64)


def _triangulate_centered_mm(
    pt_l:    np.ndarray,
    pt_r:    np.ndarray,
    k_l:     np.ndarray,
    d_l:     np.ndarray,
    k_r:     np.ndarray,
    d_r:     np.ndarray,
    p_l:     np.ndarray,
    p_r:     np.ndarray,
    t_lr_mm: np.ndarray,
) -> np.ndarray:
    ul = _undistort_pixel(pt_l, k_l, d_l)
    ur = _undistort_pixel(pt_r, k_r, d_r)
    x_h = cv2.triangulatePoints(
        np.asarray(p_l, dtype=np.float64),
        np.asarray(p_r, dtype=np.float64),
        ul.reshape(2, 1),
        ur.reshape(2, 1),
    )
    x = (x_h[:3] / x_h[3]).reshape(3)
    # Shift origin to the midpoint between the two camera centres.
    return x - 0.5 * np.asarray(t_lr_mm, dtype=np.float64).reshape(3)


# ---------------------------------------------------------------------------
# OpenCV overlay
# ---------------------------------------------------------------------------

def _draw_detections(
    image_bgr:  np.ndarray,
    detections: dict[int, np.ndarray],
) -> np.ndarray:
    out = image_bgr.copy()
    for mid, centre in detections.items():
        role  = ROLE_BY_ID.get(mid)
        color = _BGR_BY_ROLE.get(role, (180, 180, 180)) if role else (180, 180, 180)
        cx, cy = int(round(centre[0])), int(round(centre[1]))
        cv2.circle(out, (cx, cy), 8, color, 2, cv2.LINE_AA)
        label = f"ID {mid} ({role})" if role else f"ID {mid}"
        cv2.putText(out, label, (cx + 10, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, (cx + 10, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color,     1, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# 3D cylinder geometry helper
# ---------------------------------------------------------------------------

def _cylinder_surface(
    centre: np.ndarray,
    radius: float,
    depth:  float,
    n:      int = 32,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    theta    = np.linspace(0.0, 2.0 * np.pi, n)
    z_levels = np.array([0.0, depth])
    T, Zlvl  = np.meshgrid(theta, z_levels)
    X = centre[0] + radius * np.cos(T)
    Y = centre[1] + radius * np.sin(T)
    Z = centre[2] + Zlvl
    return X, Y, Z


# ---------------------------------------------------------------------------
# 3D Matplotlib plot
# ---------------------------------------------------------------------------

def _create_3D_plot_window():
    plt.ion()
    fig = plt.figure("3D Workspace Plot", figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.view_init(elev=30, azim=-60)
    plt.show(block=False)
    return fig, ax


def _flip_z(pt: np.ndarray) -> np.ndarray:
    """Negate Z so that objects below the camera (positive depth) appear below it."""
    p = np.asarray(pt, dtype=np.float64).copy()
    p[2] = -p[2]
    return p


def _rectify_corners(
    points_3d: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    """Return corners projected onto a rectangle aligned with the XY axes.

    All four corners share the same Z (mean of detected Z values).  X is taken
    from TL/BL (left side) and TR/BR (right side); Y is taken from TL/TR (top)
    and BL/BR (bottom).  This forces sides to be exactly parallel/perpendicular
    to X and Y axes regardless of small measurement noise.
    """
    present = {r: points_3d[r] for r in ["TL", "TR", "BR", "BL"] if r in points_3d}
    if len(present) < 2:
        # Not enough corners to infer a rectangle; return as-is.
        return {r: _flip_z(p) for r, p in present.items()}

    mean_z = float(np.mean([p[2] for p in present.values()]))

    # Best estimates for left/right X and top/bottom Y using available corners.
    left_roles  = [r for r in ("TL", "BL") if r in present]
    right_roles = [r for r in ("TR", "BR") if r in present]
    top_roles   = [r for r in ("TL", "TR") if r in present]
    bot_roles   = [r for r in ("BL", "BR") if r in present]

    x_left  = float(np.mean([present[r][0] for r in left_roles]))  if left_roles  else None
    x_right = float(np.mean([present[r][0] for r in right_roles])) if right_roles else None
    y_top   = float(np.mean([present[r][1] for r in top_roles]))   if top_roles   else None
    y_bot   = float(np.mean([present[r][1] for r in bot_roles]))   if bot_roles   else None

    # Fall back to the measured value when a side can't be inferred.
    result: dict[str, np.ndarray] = {}
    for role, pt in present.items():
        x = (x_left  if role in ("TL", "BL") else x_right) if (x_left is not None and x_right is not None) else pt[0]
        y = (y_top   if role in ("TL", "TR") else y_bot)   if (y_top  is not None and y_bot   is not None) else pt[1]
        result[role] = _flip_z(np.array([x, y, mean_z]))

    return result


def _update_3d_plot(
    ax,
    points_3d: dict[str, np.ndarray],
    t_lr_mm: np.ndarray,
) -> None:
    """Update the existing 3D plot window."""
    # Preserve the user's current view angle so the scene feels stationary.
    try:
        saved_elev = ax.elev
        saved_azim = ax.azim
    except AttributeError:
        saved_elev, saved_azim = 30, -60

    ax.clear()
    ax.view_init(elev=saved_elev, azim=saved_azim)

    t = np.asarray(t_lr_mm, dtype=np.float64).reshape(3)
    # Flip Z on camera positions as well.
    cam_left  = _flip_z(-0.5 * t)
    cam_right = _flip_z( 0.5 * t)

    ax.set_title("ArUco Workspace Triangulation", pad=12)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("−Z / depth (mm)")

    # Project corners onto an axis-aligned rectangle at a common Z plane.
    rect_pts = _rectify_corners(points_3d)

    # Cameras
    radius = CAMERA_DIAMETER_MM / 2.0
    for cam_centre, cam_label in [(cam_left, "Left cam"), (cam_right, "Right cam")]:
        X, Y, Z = _cylinder_surface(cam_centre, radius, CAMERA_DEPTH_MM)
        ax.plot_surface(X, Y, Z, alpha=0.40, color="steelblue", linewidth=0)
        ax.text(
            cam_centre[0], cam_centre[1], cam_centre[2] - 6.0,
            cam_label,
            color="steelblue",
            fontsize=8,
            ha="center",
        )

    # Baseline
    ax.plot(
        [cam_left[0], cam_right[0]],
        [cam_left[1], cam_right[1]],
        [cam_left[2], cam_right[2]],
        color="steelblue",
        linewidth=1.5,
        linestyle="--",
        label="Baseline",
    )

    # Workspace corners (axis-aligned rectangle)
    corner_order = ["TL", "TR", "BR", "BL"]
    corner_pts = [rect_pts[r] for r in corner_order if r in rect_pts]

    for role in corner_order:
        if role not in rect_pts:
            continue

        pt  = rect_pts[role]
        rgb = _RGB_BY_ROLE[role]

        ax.scatter(*pt, color=rgb, s=90, zorder=5)
        ax.text(
            pt[0], pt[1], pt[2] + 5.0,
            role,
            color=rgb,
            fontsize=11,
            fontweight="bold",
            ha="center",
        )

    # Workspace boundary + surface
    if len(corner_pts) == 4:
        loop = corner_pts + [corner_pts[0]]

        ax.plot(
            [p[0] for p in loop],
            [p[1] for p in loop],
            [p[2] for p in loop],
            color="darkorange",
            linewidth=2.2,
            label="Workspace boundary",
        )

        verts = [np.array([[p[0], p[1], p[2]] for p in corner_pts])]
        poly = Poly3DCollection(
            verts,
            alpha=0.18,
            facecolor="orange",
            edgecolor="none",
        )
        ax.add_collection3d(poly)

    # Rays from cameras to corners
    for role in corner_order:
        if role not in rect_pts:
            continue

        pt  = rect_pts[role]
        rgb = _RGB_BY_ROLE[role]

        for cam_c in (cam_left, cam_right):
            ax.plot(
                [cam_c[0], pt[0]],
                [cam_c[1], pt[1]],
                [cam_c[2], pt[2]],
                color=rgb,
                linewidth=0.8,
                linestyle="-",
                alpha=0.45,
            )

    # Equal-ish aspect ratio
    all_pts = list(rect_pts.values()) + [cam_left, cam_right]
    arr = np.array(all_pts)

    mid  = arr.mean(axis=0)
    span = np.ptp(arr, axis=0).max() / 2.0 + 30.0

    ax.set_xlim(mid[0] - span, mid[0] + span)
    ax.set_ylim(mid[1] - span, mid[1] + span)
    ax.set_zlim(mid[2] - span, mid[2] + span)

    ax.legend(loc="upper left")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    calib = config.load_stereo_calibration()
    if calib is None:
        print(f"[ERROR] No calibration found at {config.ACTIVE_CALIBRATION_NPZ}")
        print("Run the stereo calibration script first.")
        return

    k_l     = np.asarray(calib["left_camera_matrix"],                    dtype=np.float64)
    d_l     = np.asarray(calib["left_distortion_coefficients"],          dtype=np.float64)
    k_r     = np.asarray(calib["right_camera_matrix"],                   dtype=np.float64)
    d_r     = np.asarray(calib["right_distortion_coefficients"],         dtype=np.float64)
    p_l     = np.asarray(calib["projection_left_raw"],                   dtype=np.float64)
    p_r     = np.asarray(calib["projection_right_raw"],                  dtype=np.float64)
    t_lr_mm = np.asarray(calib["stereo_translation_left_to_right_mm"],   dtype=np.float64)

    detector = _build_detector(ARUCO_DICT_TYPE)

    with StereoCamera() as stereo:
        stereo.warmup()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, STREAM_WIDTH, STREAM_HEIGHT)
        fig_3d, ax_3d =_create_3D_plot_window()


        while True:
            ok, left, right = stereo.read_pair()
            if not ok or left is None or right is None:
                continue

            det_l = _detect(detector, left)
            det_r = _detect(detector, right)

            # Triangulate every marker visible in BOTH eyes.
            current_3d: dict[str, np.ndarray] = {}
            for mid in MARKER_IDS_CW:
                role = ROLE_BY_ID.get(mid)
                if role and mid in det_l and mid in det_r:
                    try:
                        pt = _triangulate_centered_mm(
                            det_l[mid], det_r[mid],
                            k_l, d_l, k_r, d_r,
                            p_l, p_r, t_lr_mm,
                        )
                        current_3d[role] = pt
                    except Exception as exc:
                        print(f"[WARN] Triangulation failed for ID {mid}: {exc}")

            left_view  = _draw_detections(left,  det_l)
            right_view = _draw_detections(right, det_r)

            found_roles = list(current_3d.keys())
            hud = [
                f"Stereo: {', '.join(found_roles) or 'none'}  ({len(found_roles)}/4)",
                " Q = quit",
            ]
            preview = stereo.render_preview(
                left_view, right_view,
                lines=hud,
                left_ok=(len(det_l) > 0),
                right_ok=(len(det_r) > 0),
            )
            cv2.imshow(WINDOW_NAME, preview)
            

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if current_3d:
                for role in _ROLE_NAMES:
                    if role in current_3d:
                        pt = current_3d[role]
                        print(f"  {role}: X={pt[0]:+8.2f}  Y={pt[1]:+8.2f}  Z={pt[2]:+8.2f}")
                    else:
                        print(f"  {role}: not detected")

                _update_3d_plot(ax_3d, current_3d, t_lr_mm)
                fig_3d.canvas.draw_idle()

            fig_3d.canvas.flush_events()


if __name__ == "__main__":
    main()
