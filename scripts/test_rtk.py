#!/usr/bin/env python3
"""
Test script for RTK module only. Streams pose and IMU from Point One device.
Run from project root: python scripts/test_rtk.py [--port /dev/ttyUSB0] [--view]
"""

import argparse
import math
import os
import sys
import time

# Ensure project root is on path when run as scripts/test_rtk.py
if __name__ == "__main__":
    _root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _root not in sys.path:
        sys.path.insert(0, _root)

from drive.core.rtk_controller import RTKController
from drive.core.config import DEFAULT_RTK_SERIAL_PORT

try:
    import pygame
    import pymap3d
    _VIEW_AVAILABLE = True
except (ImportError, Exception):
    _VIEW_AVAILABLE = False


def _fmt_num(x, decimals=4):
    """Format a number (or np.float64) for display; nan -> '—'."""
    if x is None:
        return "—"
    try:
        v = float(x)
        if v != v:  # nan
            return "—"
        return f"{v:.{decimals}f}".rstrip("0").rstrip(".")
    except (TypeError, ValueError):
        return str(x)


def _fmt_vec(vec, decimals=4):
    """Format a 3-vector for compact display."""
    if vec is None or not hasattr(vec, "__iter__"):
        return "—"
    parts = [_fmt_num(v, decimals) for v in list(vec)[:3]]
    if len(parts) == 3 and all(p == "—" for p in parts):
        return "—"
    return f"[{', '.join(parts)}]"


def _enu_from_lla(lat_deg, lon_deg, alt_m, lat0_deg, lon0_deg, alt0_m):
    """Convert LLA to local ENU (east, north, up) in meters. Requires pymap3d."""
    try:
        e, n, u = pymap3d.geodetic2enu(
            float(lat_deg), float(lon_deg), float(alt_m),
            float(lat0_deg), float(lon0_deg), float(alt0_m),
        )
        return (e, n, u)
    except Exception:
        return None


def _project_3d(e, n, u, az_rad, el_rad, scale, cx, cy):
    """Project ENU (e,n,u) to 2D screen with camera azimuth and elevation (rad)."""
    cos_az, sin_az = math.cos(az_rad), math.sin(az_rad)
    cos_el, sin_el = math.cos(el_rad), math.sin(el_rad)
    sx = (e * cos_az - n * sin_az) * scale + cx
    sy = cy - (u * cos_el + (e * sin_az + n * cos_az) * sin_el) * scale
    return (int(round(sx)), int(round(sy)))


def _rotation_matrix_rpy(roll, pitch, yaw):
    """Rotation matrix from roll (x), pitch (y), yaw (z) in radians. Body from world."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _orientation_from_accel_gyro(accel_xyz, gyro_xyz, dt, prev_roll, prev_pitch, prev_yaw):
    """
    Tilt from accel (gravity direction), yaw from gyro integration.
    accel in m/s² (body frame), gyro in rad/s. Returns (roll, pitch, yaw) in rad.
    """
    roll, pitch, yaw = prev_roll, prev_pitch, prev_yaw
    if accel_xyz is not None and len(accel_xyz) >= 3:
        ax, ay, az = float(accel_xyz[0]), float(accel_xyz[1]), float(accel_xyz[2])
        n = math.sqrt(ax * ax + ay * ay + az * az)
        if n > 0.1:
            # Tilt: roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2+az^2))
            roll = math.atan2(ay, az)
            pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    if gyro_xyz is not None and len(gyro_xyz) >= 3 and dt > 0:
        gz = float(gyro_xyz[2])
        yaw = prev_yaw + gz * dt
    return (roll, pitch, yaw)


# Cube: 8 vertices (local frame, half-size 1), 12 edges
_CUBE_VERTICES = [
    (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
    (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
]
_CUBE_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def _draw_cube(screen, center_enu, roll, pitch, yaw, cube_scale_m, az_rad, el_rad, scale, cx, cy, width, height):
    """Draw a 3D cube at center_enu (e,n,u) with orientation (roll, pitch, yaw) in rad."""
    R = _rotation_matrix_rpy(roll, pitch, yaw)
    ce, cn, cu = center_enu if center_enu else (0, 0, 0)
    half = cube_scale_m * 0.5
    pts = []
    for (lx, ly, lz) in _CUBE_VERTICES:
        # Rotate then translate (world ENU)
        x, y, z = lx * half, ly * half, lz * half
        rx = R[0][0] * x + R[0][1] * y + R[0][2] * z
        ry = R[1][0] * x + R[1][1] * y + R[1][2] * z
        rz = R[2][0] * x + R[2][1] * y + R[2][2] * z
        e, n, u = ce + rx, cn + ry, cu + rz
        pts.append(_project_3d(e, n, u, az_rad, el_rad, scale, cx, cy))
    face_color = (100, 140, 180)
    edge_color = (200, 220, 255)
    for i, j in _CUBE_EDGES:
        if 0 <= pts[i][0] < width and 0 <= pts[i][1] < height or 0 <= pts[j][0] < width and 0 <= pts[j][1] < height:
            pygame.draw.line(screen, edge_color, pts[i], pts[j], 2)


def _draw_view(screen, trajectory, current_enu, ref_lla, az_rad, el_rad, scale, width, height,
               cube_rpy, pose, imu):
    """Draw 3D view: trajectory, cube (accel/gyro), axes, and rich HUD."""
    cx, cy = width // 2, height // 2
    bg = (18, 18, 24)
    line_color = (80, 180, 120)
    point_color = (255, 220, 80)
    text_color = (200, 200, 210)
    screen.fill(bg)

    # Axes at origin
    axis_len = 30 * scale
    for (de, dn, du), color, label in [
        ((axis_len, 0, 0), (180, 80, 80), "E"),
        ((0, axis_len, 0), (80, 180, 80), "N"),
        ((0, 0, axis_len), (80, 80, 180), "U"),
    ]:
        p0 = _project_3d(0, 0, 0, az_rad, el_rad, scale, cx, cy)
        p1 = _project_3d(de, dn, du, az_rad, el_rad, scale, cx, cy)
        pygame.draw.line(screen, color, p0, p1, 2)
        try:
            font = pygame.font.Font(None, 24)
            surf = font.render(label, True, color)
            screen.blit(surf, (p1[0] + 4, p1[1] - 8))
        except Exception:
            pass

    # Trajectory
    if len(trajectory) >= 2:
        points = [_project_3d(e, n, u, az_rad, el_rad, scale, cx, cy) for (e, n, u) in trajectory]
        for i in range(len(points) - 1):
            pygame.draw.line(screen, line_color, points[i], points[i + 1], 2)

    # Cube (mirror of accel tilt + gyro rotation) at current position
    cube_scale_m = max(8.0, (axis_len / scale) * 0.6)  # 4x larger cube
    if current_enu is not None and cube_rpy is not None:
        roll, pitch, yaw = cube_rpy
        _draw_cube(screen, current_enu, roll, pitch, yaw, cube_scale_m, az_rad, el_rad, scale, cx, cy, width, height)
    elif cube_rpy is not None:
        _draw_cube(screen, (0, 0, 0), cube_rpy[0], cube_rpy[1], cube_rpy[2], cube_scale_m, az_rad, el_rad, scale, cx, cy, width, height)

    # Current position dot
    if current_enu is not None:
        px, py = _project_3d(current_enu[0], current_enu[1], current_enu[2], az_rad, el_rad, scale, cx, cy)
        if 0 <= px < width and 0 <= py < height:
            pygame.draw.circle(screen, point_color, (px, py), 6)
            pygame.draw.circle(screen, (255, 255, 255), (px, py), 6, 1)

    # Rich HUD
    try:
        font = pygame.font.Font(None, 20)
        y_line = 8
        line_h = 18

        def _blit(text, color=text_color):
            nonlocal y_line
            s = font.render(text, True, color)
            screen.blit(s, (8, y_line))
            y_line += line_h

        _blit("--- Pose (GNSS) ---", (150, 180, 150))
        if pose:
            _blit(f"  solution_type: {pose.get('solution_type', '—')}")
            lla = pose.get("lla_deg")
            if lla is not None and hasattr(lla, "__iter__") and len(lla) >= 3:
                _blit(f"  LLA (deg): [{float(lla[0]):.6f}, {float(lla[1]):.6f}, {float(lla[2]):.3f}]")
            std = pose.get("position_std_enu_m")
            if std is not None and hasattr(std, "__iter__"):
                _blit(f"  position_std_enu_m: [{float(std[0]):.0f}, {float(std[1]):.0f}, {float(std[2]):.0f}]")
            gps = pose.get("gps_time")
            if gps is not None:
                _blit(f"  gps_time: {gps}")
        else:
            _blit("  (no pose yet)")

        _blit("--- IMU (cube = accel tilt + gyro yaw) ---", (150, 180, 150))
        if imu:
            acc = imu.get("accel_xyz")
            if acc is not None and hasattr(acc, "__iter__") and len(acc) >= 3:
                _blit(f"  acc (m/s²): [{float(acc[0]):.3f}, {float(acc[1]):.3f}, {float(acc[2]):.3f}]", (220, 200, 120))
            gyro = imu.get("gyro_xyz")
            if gyro is not None and hasattr(gyro, "__iter__") and len(gyro) >= 3:
                _blit(f"  gyro (rad/s): [{float(gyro[0]):.4f}, {float(gyro[1]):.4f}, {float(gyro[2]):.4f}]", (120, 200, 220))
            p1 = imu.get("p1_time")
            if p1 is not None:
                _blit(f"  p1_time: {float(p1):.2f} s")
        else:
            _blit("  (no IMU yet)")

        _blit("--- Trajectory ---", (150, 180, 150))
        _blit(f"  points: {len(trajectory)}  ref_lla: {'yes' if ref_lla else 'no'}")

        _blit("--- View ---", (150, 180, 150))
        _blit("  Az/El: drag  Scale: +/-  Quit: Esc/close")

        if cube_rpy is not None:
            r, p, y = cube_rpy
            _blit(f"  cube r,p,y (deg): [{math.degrees(r):.1f}, {math.degrees(p):.1f}, {math.degrees(y):.1f}]")
    except Exception:
        pass

    pygame.display.flip()


def main():
    parser = argparse.ArgumentParser(description="Test RTK module (pose + IMU stream)")
    parser.add_argument(
        "--port",
        type=str,
        default=DEFAULT_RTK_SERIAL_PORT,
        help=f"Serial port for RTK device (default: {DEFAULT_RTK_SERIAL_PORT})",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=2.0,
        help="Print rate in Hz (default: 2)",
    )
    parser.add_argument(
        "--view",
        action="store_true",
        default=True,
        help="Show 3D trajectory view (default: True)",
    )
    parser.add_argument(
        "--no-view",
        action="store_false",
        dest="view",
        help="Disable 3D view (console only)",
    )
    args = parser.parse_args()

    use_view = args.view and _VIEW_AVAILABLE
    if args.view and not _VIEW_AVAILABLE:
        print("[Test] 3D view skipped: pygame or pymap3d not available.")

    rtk = RTKController(serial_port=args.port, enabled=True)
    if not rtk.initialize():
        sys.exit(1)

    # 3D view state
    ref_lla = None  # (lat0, lon0, alt0)
    trajectory = []
    current_enu = None
    az_rad = math.radians(45)
    el_rad = math.radians(25)
    scale = 3.0
    view_size = (800, 600)
    screen = None
    mouse_down = False
    last_mouse = (0, 0)
    # Cube orientation: tilt from accel, yaw from gyro integration (rad)
    cube_roll, cube_pitch, cube_yaw = 0.0, 0.0, 0.0
    last_imu_time = None

    if use_view:
        try:
            pygame.init()
            pygame.display.set_caption("RTK 3D — trajectory (ENU)")
            screen = pygame.display.set_mode(view_size, pygame.RESIZABLE)
        except Exception as e:
            print(f"[Test] Pygame init failed: {e}")
            use_view = False

    interval = 1.0 / args.hz
    next_print = time.monotonic()
    try:
        while True:
            # Pygame events (quit, resize, mouse for camera)
            if use_view and screen is not None:
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    if ev.type == pygame.KEYDOWN:
                        if ev.key == pygame.K_ESCAPE:
                            raise KeyboardInterrupt
                        if ev.key in (pygame.K_PLUS, pygame.K_EQUALS):
                            scale = min(scale * 1.2, 100.0)
                        if ev.key == pygame.K_MINUS:
                            scale = max(scale / 1.2, 0.5)
                    if ev.type == pygame.VIDEORESIZE:
                        view_size = (ev.w, ev.h)
                        screen = pygame.display.set_mode(view_size, pygame.RESIZABLE)
                    if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                        mouse_down = True
                        last_mouse = ev.pos
                    if ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                        mouse_down = False
                    if ev.type == pygame.MOUSEMOTION and mouse_down:
                        dx = ev.pos[0] - last_mouse[0]
                        dy = ev.pos[1] - last_mouse[1]
                        last_mouse = ev.pos
                        az_rad -= math.radians(dx * 0.5)
                        el_rad = max(-1.4, min(1.4, el_rad + math.radians(dy * 0.5)))

            rtk.update()
            pose = rtk.get_latest_pose()
            imu = rtk.get_latest_imu()
            now_loop = time.monotonic()

            if pose:
                lla = pose.get("lla_deg")
                if lla is not None and hasattr(lla, "__iter__") and len(lla) >= 3:
                    lat, lon, alt = float(lla[0]), float(lla[1]), float(lla[2])
                    if ref_lla is None:
                        ref_lla = (lat, lon, alt)
                    enu = _enu_from_lla(lat, lon, alt, ref_lla[0], ref_lla[1], ref_lla[2])
                    if enu is not None:
                        current_enu = enu
                        if not trajectory or trajectory[-1] != current_enu:
                            trajectory.append(current_enu)
                            if len(trajectory) > 10000:
                                trajectory.pop(0)

            # Cube orientation: tilt from accel, yaw from gyro integration
            if imu:
                acc = imu.get("accel_xyz")
                gyro = imu.get("gyro_xyz")
                dt = (now_loop - last_imu_time) if last_imu_time is not None else 0.02
                last_imu_time = now_loop
                cube_roll, cube_pitch, cube_yaw = _orientation_from_accel_gyro(
                    acc, gyro, dt, cube_roll, cube_pitch, cube_yaw
                )
            cube_rpy = (cube_roll, cube_pitch, cube_yaw)

            if use_view and screen is not None:
                w, h = screen.get_size()
                _draw_view(screen, trajectory, current_enu, ref_lla, az_rad, el_rad, scale, w, h,
                           cube_rpy, pose, imu)

            now = time.monotonic()
            if now >= next_print:
                next_print = now + interval
                if pose:
                    sol = pose.get("solution_type", "—")
                    lla = _fmt_vec(pose.get("lla_deg"), 6)
                    gps = pose.get("gps_time")
                    gps_str = str(gps).split("(")[-1].rstrip(")") if gps else "—"
                    std = _fmt_vec(pose.get("position_std_enu_m"), 0)
                    ypr = _fmt_vec(pose.get("ypr_deg"), 2)
                    vel = _fmt_vec(pose.get("velocity_body_mps"), 2)
                    print(f"  Pose  {sol:16}  LLA(°) {lla}  std_enu(m) {std}")
                    print(f"        GPS {gps_str}  ypr(°) {ypr}  v_body(m/s) {vel}")
                else:
                    print("  Pose  (no data yet)")
                if imu:
                    acc = _fmt_vec(imu.get("accel_xyz"), 4)
                    gyro = _fmt_vec(imu.get("gyro_xyz"), 5)
                    p1 = imu.get("p1_time")
                    p1_str = f"{float(p1):.2f}s" if p1 is not None else "—"
                    print(f"  IMU   acc(m/s²) {acc}  gyro(rad/s) {gyro}  P1 {p1_str}")
                else:
                    print("  IMU   (no data yet; need SDK-AP firmware for IMU)")
                print()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[Test] Interrupted.")
    finally:
        rtk.stop()


if __name__ == "__main__":
    main()
