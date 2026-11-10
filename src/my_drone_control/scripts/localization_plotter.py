#!/usr/bin/env python3
"""Plot localization sensors against isolated evaluation ground truth.

Streams: GPS (5 Hz), VO (30 Hz), IMU (100 Hz), truth (100 Hz), EKF (100 Hz).
EKF output is overlaid on every subplot for easy comparison.
The EKF 2-σ covariance ellipse is drawn on the Horizontal Track (XY) subplot.
"""

import bisect
import math
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


TOPICS = ('gps', 'vo', 'imu', 'truth')
ALL_STREAMS = ('gps', 'vo', 'imu', 'truth', 'ekf')
EXPECTED_FRAMES = {
    'gps': ('map', None),
    'vo': ('vo_odom', 'base_link'),
    'imu': ('base_link', None),
    'truth': ('map', 'base_link'),
    'ekf': ('map', 'base_link'),
}
STALE_AFTER_SIM_S  = {'gps': 1.0, 'vo': 0.5, 'imu': 0.5, 'truth': 0.5, 'ekf': 1.0}
STALE_AFTER_WALL_S = {'gps': 8.0, 'vo': 5.0, 'imu': 5.0, 'truth': 5.0, 'ekf': 8.0}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def covariance_ellipse_params(cov_xx, cov_xy, cov_yy, n_sigma=2.0):
    """Return (width, height, angle_deg) of the n-sigma ellipse from a 2×2 covariance."""
    mat = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
    try:
        eigvals, eigvecs = np.linalg.eigh(mat)
        eigvals = np.maximum(eigvals, 0.0)
        order = np.argsort(eigvals)[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]
        angle = math.degrees(math.atan2(eigvecs[1, 0], eigvecs[0, 0]))
        width  = 2.0 * n_sigma * math.sqrt(eigvals[0])
        height = 2.0 * n_sigma * math.sqrt(eigvals[1])
        return width, height, angle
    except Exception:
        return 0.0, 0.0, 0.0


class LocalizationPlotter(Node):
    def __init__(self):
        super().__init__('localization_plotter')
        self.declare_parameter('window_s', 120.0)
        self.window_s = self.get_parameter('window_s').value
        self.lock = threading.Lock()
        self.start_wall = time.monotonic()
        self.plot_time_offset = 0.0
        self.last_stamp = {name: None for name in ALL_STREAMS}
        self.latest_plot_time = None
        self.samples = {name: deque(maxlen=12000) for name in ALL_STREAMS}
        self.arrivals = {name: deque(maxlen=2000) for name in ALL_STREAMS}
        self.frames = {name: ('-', '-') for name in ALL_STREAMS}
        self.status = {name: 'WAITING' for name in ALL_STREAMS}
        self.frame_warnings = set()
        self.last_summary_wall = 0.0
        self.all_sensors_ready = False
        self.animation = None

        self.ekf_covariances = deque(maxlen=12000)

        self.create_subscription(
            PoseWithCovarianceStamped, '/localization/gps', self.on_gps,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/localization/vo', self.on_vo, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/localization/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/evaluation/ground_truth', self.on_truth,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/localization/ekf', self.on_ekf, qos_profile_sensor_data)
        self.create_timer(
            1.0, self.report_diagnostics,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            'Plotter inputs: GPS 5 Hz, VO 30 Hz, IMU 100 Hz, truth 100 Hz, EKF. '
            'Ground truth is used only by this visualization node.')

    def record(self, name, message, x, y, z, yaw=None, gyro_z=None):
        stamp = stamp_seconds(message.header.stamp)
        frame_id = message.header.frame_id or '<empty>'
        child_frame_id = getattr(message, 'child_frame_id', '-') or '-'
        arrival = time.monotonic()
        reset_detected = False
        with self.lock:
            prev_sensor_stamp = self.last_stamp.get(name)
            if prev_sensor_stamp is not None and stamp < prev_sensor_stamp - 2.0:
                prev_latest = self.latest_plot_time or 0.0
                candidate_offset = prev_latest + 0.01 - stamp
                if candidate_offset > self.plot_time_offset:
                    self.plot_time_offset = candidate_offset
                    reset_detected = True
            self.last_stamp[name] = stamp
            plot_time = stamp + self.plot_time_offset
            if self.latest_plot_time is None or plot_time > self.latest_plot_time:
                self.latest_plot_time = plot_time
            self.samples[name].append((stamp, x, y, z, yaw, gyro_z,
                                       arrival - self.start_wall, plot_time))
            self.arrivals[name].append((stamp, arrival))
            self.frames[name] = (frame_id, child_frame_id)
        if reset_detected:
            self.get_logger().warning(
                f'{name.upper()} simulation clock reset at {stamp:.2f}s; '
                f'plot timeline will continue across the reset (offset={self.plot_time_offset:.2f}s)')
        return plot_time

    def on_gps(self, message):
        position = message.pose.pose.position
        self.record('gps', message, position.x, position.y, position.z)

    def on_vo(self, message):
        pose = message.pose.pose
        position = pose.position
        self.record(
            'vo', message, position.x, position.y, position.z,
            quaternion_yaw(pose.orientation))

    def on_imu(self, message):
        self.record(
            'imu', message, 0.0, 0.0, message.angular_velocity.z,
            gyro_z=message.angular_velocity.z)

    def on_truth(self, message):
        pose = message.pose.pose
        position = pose.position
        self.record(
            'truth', message, position.x, position.y, position.z,
            quaternion_yaw(pose.orientation))

    def on_ekf(self, message):
        pose = message.pose.pose
        position = pose.position
        yaw = quaternion_yaw(pose.orientation)

        cov = message.pose.covariance
        cov_xx = float(cov[0]) if len(cov) >= 36 else 1.0
        cov_xy = float(cov[1]) if len(cov) >= 36 else 0.0
        cov_yy = float(cov[7]) if len(cov) >= 36 else 1.0

        plot_time = self.record('ekf', message, position.x, position.y, position.z, yaw=yaw)
        with self.lock:
            self.ekf_covariances.append((plot_time, cov_xx, cov_xy, cov_yy))

    @staticmethod
    def rate_hz(samples, now_wall):
        recent = [item for item in samples if now_wall - item[1] <= 10.0]
        if len(recent) < 2:
            return None, None
        stamp_span = recent[-1][0] - recent[0][0]
        wall_span = recent[-1][1] - recent[0][1]
        stamp_rate = (len(recent) - 1) / stamp_span if stamp_span > 0 else None
        wall_rate = (len(recent) - 1) / wall_span if wall_span > 0 else None
        return stamp_rate, wall_rate

    @staticmethod
    def interpolate_truth(truth, truth_stamps, plot_time):
        index = bisect.bisect_left(truth_stamps, plot_time)
        if index == 0 or index == len(truth):
            return None
        before, after = truth[index - 1], truth[index]
        span = after[7] - before[7]
        if span <= 0:
            return None
        ratio = (plot_time - before[7]) / span
        return tuple(before[i] + ratio * (after[i] - before[i]) for i in (1, 2, 3))

    def residual_summary(self, sensor, truth):
        if len(truth) < 2 or len(sensor) < 2:
            return 'not ready'
        truth_stamps = [sample[7] for sample in truth]
        first_stamp = truth[0][7]
        last_stamp = truth[-1][7]
        window_start = last_stamp - 10.0
        errors = []
        for sample in sensor:
            if (sample[7] < max(first_stamp, window_start) or
                    sample[7] > last_stamp):
                continue
            reference = self.interpolate_truth(truth, truth_stamps, sample[7])
            if reference is not None:
                errors.append(tuple(sample[i] - reference[i] for i in (1, 2, 3)))
        if not errors:
            return 'not synchronized yet'
        rms = tuple(math.sqrt(sum(error[i] ** 2 for error in errors) / len(errors))
                    for i in range(3))
        latest = errors[-1]
        return ('RMS xyz=({:.2f}, {:.2f}, {:.2f}) m; latest=({:.2f}, {:.2f}, {:.2f}) m'
                .format(*rms, *latest))

    def report_diagnostics(self):
        try:
            now_sim = self.get_clock().now().nanoseconds * 1e-9
            now_wall = time.monotonic()
            with self.lock:
                samples = {name: list(data) for name, data in self.samples.items()}
                arrivals = {name: list(data) for name, data in self.arrivals.items()}
                frames = dict(self.frames)
                latest_sim = self.latest_plot_time

            ref_sim = latest_sim if latest_sim is not None else now_sim

            for name in ALL_STREAMS:
                latest = samples[name][-1] if samples[name] else None
                age = (ref_sim - latest[7]) if latest else None
                wall_age = (now_wall - arrivals[name][-1][1]) if arrivals[name] else None
                recent = [item for item in arrivals[name] if now_wall - item[1] <= 10.0]
                stamp_stalled = (
                    len(recent) >= 2 and
                    recent[-1][1] - recent[0][1] >= 5.0 and
                    recent[-1][0] - recent[0][0] <= 1e-6)
                state = ('WAITING' if latest is None else
                         'STAMP STOP' if stamp_stalled else
                         'STALE' if (
                             (age is not None and age > STALE_AFTER_SIM_S[name]) or
                             (wall_age is not None and wall_age > STALE_AFTER_WALL_S[name]))
                         else 'LIVE')
                old_state = self.status[name]
                if state != old_state:
                    self.status[name] = state
                    if state == 'LIVE':
                        self.get_logger().info(f'{name.upper()} stream live: frame={frames[name][0]}')
                    else:
                        self.get_logger().warning(
                            f'{name.upper()} {old_state} -> {state}' +
                            (f' (sim age {age:.2f}s, wall age {wall_age:.2f}s)'
                             if age is not None and wall_age is not None else ''))

                expected_frame, expected_child = EXPECTED_FRAMES[name]
                frame, child = frames[name]
                if latest and (frame != expected_frame or
                               (expected_child is not None and child != expected_child)):
                    if name not in self.frame_warnings:
                        self.frame_warnings.add(name)
                        self.get_logger().warning(
                            f'{name.upper()} frame contract mismatch: '
                            f'{frame}/{child}; expected {expected_frame}/{expected_child or "(no child)"}')

            sensors_ready = all(self.status[name] == 'LIVE' for name in TOPICS)
            if sensors_ready != self.all_sensors_ready:
                self.all_sensors_ready = sensors_ready
                if sensors_ready:
                    self.get_logger().info(
                        'READY: GPS, VO, IMU, and evaluation truth streams are live; '
                        'plot history is collecting.')
                elif any(self.status[name] == 'STALE' for name in TOPICS):
                    self.get_logger().warning('Sensor readiness lost; see stream status above')

            if now_wall - self.last_summary_wall < 15.0:
                return
            self.last_summary_wall = now_wall
            lines = ['Sensor diagnostics (stamp Hz uses message timestamps; wall Hz uses arrivals):']
            for name in ALL_STREAMS:
                stamp_rate, wall_rate = self.rate_hz(arrivals[name], now_wall)
                rate_text = ('no stamp advance' if stamp_rate is None and wall_rate is not None else
                             'warming up' if stamp_rate is None else
                             f'{stamp_rate:.1f} / {wall_rate:.1f}')
                latest = samples[name][-1] if samples[name] else None
                age_text = f'{ref_sim - latest[7]:.2f}s' if latest else 'n/a'
                frame, child = frames[name]
                lines.append(
                    f'  {name.upper():5} {self.status[name]:7} rate(stamp/wall)={rate_text:>13} Hz '
                    f'age={age_text:>6} frame={frame}/{child}')

            truth = samples['truth']
            if truth:
                lines.append('  GPS residual vs truth: ' +
                             self.residual_summary(samples['gps'], truth))
                lines.append('  VO  residual vs truth: ' +
                             self.residual_summary(samples['vo'], truth) +
                             ' (raw vo_odom coordinates)')
                lines.append('  EKF residual vs truth: ' +
                             self.residual_summary(samples['ekf'], truth))
            gyro = [sample[5] for sample in samples['imu']
                    if sample[7] >= ref_sim - 10.0 and sample[5] is not None]
            if gyro:
                mean = sum(gyro) / len(gyro)
                deviation = math.sqrt(sum((value - mean) ** 2 for value in gyro) / len(gyro))
                lines.append(
                    f'  IMU gyro_z last 10 sim s: mean={mean:+.5f}, '
                    f'std={deviation:.5f} rad/s (mean includes real turning motion)')
            self.get_logger().info('\n' + '\n'.join(lines))
        except Exception as e:
            self.get_logger().error(f'Diagnostics reporting error: {e}')

    def show_plots(self):
        figure, axes = plt.subplots(3, 2, figsize=(14, 9), constrained_layout=True)
        axes = axes.flatten()
        ax_xy, ax_x, ax_y, ax_z, ax_yaw, ax_gyro = axes
        lines = {}

        # --- Horizontal track (XY) ---
        lines['xy_truth'], = ax_xy.plot([], [], label='Ground truth', linewidth=2)
        lines['xy_gps'],   = ax_xy.plot([], [], '.', label='GPS', alpha=0.65)
        lines['xy_vo'],    = ax_xy.plot([], [], label='VO (vo_odom)', linewidth=1)
        lines['xy_ekf'],   = ax_xy.plot([], [], label='EKF', linewidth=2,
                                         color='red', zorder=5)

        ekf_ellipse = Ellipse((0, 0), width=0, height=0, angle=0,
                              edgecolor='red', facecolor='none',
                              linewidth=1.5, linestyle='--', zorder=6)
        ax_xy.add_patch(ekf_ellipse)
        ax_xy.set(title='Horizontal track', xlabel='ENU x [m]', ylabel='ENU y [m]')
        ax_xy.set_aspect('equal', adjustable='box')

        # --- X, Y, Z position subplots ---
        for key, axis, component, title in (
                ('x', ax_x, 1, 'X position'),
                ('y', ax_y, 2, 'Y position'),
                ('z', ax_z, 3, 'Altitude')):
            for name, label in (('truth', 'Ground truth'), ('gps', 'GPS'), ('vo', 'VO')):
                lines[f'{key}_{name}'], = axis.plot([], [], label=label)
            lines[f'{key}_ekf'], = axis.plot([], [], label='EKF', color='red',
                                              linewidth=2, zorder=5)
            axis.set(title=title, xlabel='Time from latest sample [s]', ylabel='Position [m]')

        # --- Yaw subplot ---
        for name, label in (('truth', 'Ground truth'), ('vo', 'VO')):
            lines[f'yaw_{name}'], = ax_yaw.plot([], [], label=label)
        lines['yaw_ekf'], = ax_yaw.plot([], [], label='EKF', color='red',
                                         linewidth=2, zorder=5)
        ax_yaw.set(title='Yaw', xlabel='Time from latest sample [s]', ylabel='Yaw [rad]')
        ax_yaw.set_ylim(-math.pi - 0.2, math.pi + 0.2)

        # --- Yaw-rate subplot ---
        lines['gyro_imu'],   = ax_gyro.plot([], [], label='IMU gyro Z')
        lines['gyro_truth'], = ax_gyro.plot([], [], label='Ground-truth yaw rate')
        lines['gyro_ekf'],   = ax_gyro.plot([], [], label='EKF yaw rate',
                                              color='red', linewidth=1.5, zorder=5)
        ax_gyro.set(title='Yaw rate', xlabel='Time from latest sample [s]', ylabel='Rate [rad/s]')

        for axis in (ax_x, ax_y, ax_z, ax_yaw, ax_gyro):
            axis.grid(True, alpha=0.25)
            axis.legend(loc='upper left')
        ax_xy.legend(loc='best')

        status_title = figure.suptitle(
            'Localization sensor characterization (ground truth is plot-only)')

        def update(_frame):
            try:
                with self.lock:
                    data = {name: list(rows) for name, rows in self.samples.items()}
                    ekf_covs = list(self.ekf_covariances)
                    status_dict = dict(self.status)
                latest_plot_times = [rows[-1][7] for rows in data.values() if rows]
                if not latest_plot_times:
                    return tuple(lines.values()) + (ekf_ellipse,)

                end = max(latest_plot_times)
                start = end - self.window_s
                earliest_plot_times = [rows[0][7] for rows in data.values() if rows]
                accumulated_s = max(0.0, end - min(earliest_plot_times))
                view_span_s = min(self.window_s, max(10.0, accumulated_s))

                status_text = ' | '.join(
                    f'{name.upper()} {status_dict[name]}' for name in TOPICS)
                ekf_state = status_dict.get('ekf', 'WAITING')
                status_title.set_text(
                    f'Localization streams | simulation elapsed {end:.1f}s | '
                    f'{status_text} | EKF {ekf_state}')

                def window(name):
                    return [sample for sample in data[name]
                            if start <= sample[7] <= end]

                # --- GPS / VO / truth ---
                for name in ('truth', 'gps', 'vo'):
                    rows = window(name)
                    lines[f'xy_{name}'].set_data(
                        [row[1] for row in rows], [row[2] for row in rows])
                    for key, component in (('x', 1), ('y', 2), ('z', 3)):
                        lines[f'{key}_{name}'].set_data(
                            [row[7] - end for row in rows], [row[component] for row in rows])
                    if name != 'gps':
                        yaw_rows = [row for row in rows if row[4] is not None]
                        lines[f'yaw_{name}'].set_data(
                            [row[7] - end for row in yaw_rows],
                            [row[4] for row in yaw_rows])

                # --- EKF track ---
                ekf_rows = window('ekf')
                if ekf_rows:
                    # FIX: Use ekf_rows instead of rows
                    lines['xy_ekf'].set_data(
                        [row[1] for row in ekf_rows], [row[2] for row in ekf_rows])
                    for key, component in (('x', 1), ('y', 2), ('z', 3)):
                        lines[f'{key}_ekf'].set_data(
                            [row[7] - end for row in ekf_rows], [row[component] for row in ekf_rows])
                    ekf_yaw_rows = [row for row in ekf_rows if row[4] is not None]
                    lines['yaw_ekf'].set_data(
                        [row[7] - end for row in ekf_yaw_rows],
                        [row[4] for row in ekf_yaw_rows])
                else:
                    lines['xy_ekf'].set_data([], [])
                    lines['x_ekf'].set_data([], [])
                    lines['y_ekf'].set_data([], [])
                    lines['z_ekf'].set_data([], [])
                    lines['yaw_ekf'].set_data([], [])

                # EKF yaw rate
                ekf_rate = []
                for before, after in zip(ekf_rows, ekf_rows[1:]):
                    dt = after[7] - before[7]
                    if (0 < dt <= 0.5 and
                            before[4] is not None and after[4] is not None):
                        ekf_rate.append((after[7] - end,
                                         wrap_angle(after[4] - before[4]) / dt))
                lines['gyro_ekf'].set_data(
                    [item[0] for item in ekf_rate],
                    [item[1] for item in ekf_rate])

                # --- EKF covariance ellipse ---
                win_covs = [c for c in ekf_covs if start <= c[0] <= end]
                if win_covs and ekf_rows:
                    latest_cov = win_covs[-1]
                    latest_ekf = ekf_rows[-1]
                    cx, cy_pos = latest_ekf[1], latest_ekf[2]
                    cov_xx, cov_xy, cov_yy = latest_cov[1], latest_cov[2], latest_cov[3]
                    if math.isfinite(cx) and math.isfinite(cy_pos) and math.isfinite(cov_xx) and math.isfinite(cov_yy):
                        w, h, ang = covariance_ellipse_params(cov_xx, cov_xy, cov_yy)
                        ekf_ellipse.set_center((cx, cy_pos))
                        ekf_ellipse.set_width(max(w, 0.05))
                        ekf_ellipse.set_height(max(h, 0.05))
                        ekf_ellipse.set_angle(ang)
                        ekf_ellipse.angle = ang
                        ekf_ellipse.set_visible(True)
                    else:
                        ekf_ellipse.set_visible(False)
                else:
                    ekf_ellipse.set_visible(False)

                # --- IMU gyro ---
                imu_rows = window('imu')
                lines['gyro_imu'].set_data(
                    [row[7] - end for row in imu_rows], [row[5] for row in imu_rows])

                # --- Truth yaw rate ---
                truth_rows = window('truth')
                truth_rate = []
                for before, after in zip(truth_rows, truth_rows[1:]):
                    dt = after[7] - before[7]
                    if (0 < dt <= 0.1 and before[4] is not None and after[4] is not None):
                        truth_rate.append((after[7] - end,
                                           wrap_angle(after[4] - before[4]) / dt))
                lines['gyro_truth'].set_data(
                    [item[0] for item in truth_rate], [item[1] for item in truth_rate])

                for axis in axes[1:]:
                    axis.set_xlim(-view_span_s, 0)
                    axis.relim()
                    axis.autoscale_view(scalex=False, scaley=True)

                xy_positions = [
                    (row[1], row[2])
                    for name in ('truth', 'gps', 'vo', 'ekf')
                    for row in window(name)
                    if math.isfinite(row[1]) and math.isfinite(row[2])
                ]
                if xy_positions:
                    x_values, y_values = zip(*xy_positions)
                    min_x, max_x = min(x_values), max(x_values)
                    min_y, max_y = min(y_values), max(y_values)
                    pad_x = max((max_x - min_x) * 0.05, 0.5)
                    pad_y = max((max_y - min_y) * 0.05, 0.5)
                    ax_xy.set_xlim(min_x - pad_x, max_x + pad_x)
                    ax_xy.set_ylim(min_y - pad_y, max_y + pad_y)

                return tuple(lines.values()) + (ekf_ellipse,)
            except Exception as e:
                self.get_logger().error(f'Plot update error: {e}')
                return tuple(lines.values()) + (ekf_ellipse,)

        self.animation = FuncAnimation(figure, update, interval=250, blit=False,
                                       cache_frame_data=False)
        plt.show()
        return self.animation


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationPlotter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    backend = plt.get_backend().lower()
    headless_backends = {'agg', 'cairo', 'pdf', 'pgf', 'ps', 'svg', 'template'}
    spin_thread = None
    anim = None
    try:
        if backend in headless_backends:
            node.get_logger().warning(
                f'Matplotlib backend "{backend}" has no interactive window; '
                'continuing with terminal diagnostics only.')
            executor.spin()
        else:
            def spin_worker():
                try:
                    executor.spin()
                except Exception as e:
                    if rclpy.ok():
                        node.get_logger().error(f'Executor spin failed: {e}')

            spin_thread = threading.Thread(target=spin_worker, daemon=True)
            spin_thread.start()
            anim = node.show_plots()
    except KeyboardInterrupt:
        pass
    finally:
        del anim
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()