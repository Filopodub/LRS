#!/usr/bin/env python3
"""Eight-state Extended Kalman Filter (EKF) node for drone localization."""

import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class EKFNode(Node):
    N = 8  # State dimension: [x, y, z, vx, vy, vz, yaw, gyro_bias]

    def __init__(self):
        super().__init__('ekf_node')

        self.declare_parameter('q_pos', 0.01)
        self.declare_parameter('q_vel', 0.5)
        self.declare_parameter('q_yaw', 0.005)
        self.declare_parameter('q_bias', 1e-5)
        self.declare_parameter('r_gps_xy', 0.36)
        self.declare_parameter('r_gps_z', 0.81)
        self.declare_parameter('r_vo_yaw', 2.74e-5)
        self.declare_parameter('r_vo_vel', 0.0009)
        self.declare_parameter('fuse_vo_position', False)
        self.declare_parameter('r_vo_pos', 0.25)

        p = lambda name: self.get_parameter(name).value

        self.Q_rate = np.diag([
            p('q_pos'), p('q_pos'), p('q_pos'),
            p('q_vel'), p('q_vel'), p('q_vel'),
            p('q_yaw'),
            p('q_bias'),
        ])

        self.R_gps_fallback = np.diag([p('r_gps_xy'), p('r_gps_xy'), p('r_gps_z')])
        self.R_vo_yaw = np.array([[p('r_vo_yaw')]])
        self.R_vo_vel = np.eye(3) * p('r_vo_vel')
        self.fuse_vo_position = p('fuse_vo_position')
        self.R_vo_pos = np.eye(3) * p('r_vo_pos')

        self.x = np.zeros(self.N)
        self.P = np.eye(self.N)
        self.initialised = False

        self.first_gps = None
        self.first_vo_yaw = None
        self.first_vo_pos = None
        self.vo_offset = None

        self.last_imu_stamp = -1.0
        self.last_gps_stamp = -1.0
        self.last_vo_stamp = -1.0
        self.latest_stamp = None

        self.pub_odom = self.create_publisher(Odometry, '/localization/ekf', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/localization/ekf_pose', 10)

        self.create_subscription(
            Imu, '/localization/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/localization/gps', self.on_gps,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/localization/vo', self.on_vo, qos_profile_sensor_data)

        self.get_logger().info('EKF Node initialized. Waiting for sensor data...')

    def try_initialise(self):
        if self.initialised:
            return

        # Inicializácia hneď po príchode prvého GPS merania
        if self.first_gps is not None:
            self.x = np.zeros(self.N)
            self.x[0:3] = self.first_gps
            if self.first_vo_yaw is not None:
                self.x[6] = wrap(self.first_vo_yaw)
            else:
                self.x[6] = 0.0  # Predvolený yaw, kým nedorazí VO

            gps_std_xy = math.sqrt(max(self.R_gps_fallback[0, 0], 1e-4))
            gps_std_z = math.sqrt(max(self.R_gps_fallback[2, 2], 1e-4))
            self.P = np.diag([
                gps_std_xy**2, gps_std_xy**2, gps_std_z**2,
                1.0, 1.0, 1.0,
                (math.pi / 4.0)**2,
                (0.1)**2,
            ])
            self.initialised = True
            self.get_logger().info(
                f'EKF Initialised: Pos=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f}) m')

    def on_imu(self, msg):
        t = stamp_to_sec(msg.header.stamp)

        if self.last_imu_stamp > 0.0 and t < self.last_imu_stamp - 0.5:
            self.last_imu_stamp = t
            return
        if self.last_imu_stamp < 0.0:
            self.last_imu_stamp = t
            return

        dt = t - self.last_imu_stamp
        if dt <= 0.0:
            return
        self.last_imu_stamp = t

        # Skús inicializovať, ak ešte nie je inicializovaný
        if not self.initialised:
            self.try_initialise()

        if not self.initialised:
            return

        dt = min(dt, 0.1)
        gyro_z = msg.angular_velocity.z
        self.latest_stamp = msg.header.stamp
        self.predict(gyro_z, dt)
        self.publish(self.latest_stamp)
    

    def predict(self, gyro_z, dt):
        x = self.x
        vx, vy, vz = x[3], x[4], x[5]
        yaw = x[6]
        bias = x[7]

        gyro_z_corrected = gyro_z - bias

        x_new = np.empty(self.N)
        x_new[0] = x[0] + vx * dt
        x_new[1] = x[1] + vy * dt
        x_new[2] = x[2] + vz * dt
        x_new[3] = vx
        x_new[4] = vy
        x_new[5] = vz
        x_new[6] = wrap(yaw + gyro_z_corrected * dt)
        x_new[7] = bias

        F = np.eye(self.N)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        F[6, 7] = -dt

        self.x = x_new
        self.P = F @ self.P @ F.T + self.Q_rate * dt

    def correct(self, z, h, H, R):
        innov = z - h
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warning('EKF update skipped: S matrix inversion failed')
            return

        self.x = self.x + K @ innov
        self.x[6] = wrap(self.x[6])

        I_KH = np.eye(self.N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), 1e-9))

    def on_gps(self, msg):
        t = stamp_to_sec(msg.header.stamp)

        if self.last_gps_stamp > 0.0 and t < self.last_gps_stamp - 0.5:
            self.last_gps_stamp = t
            return
        self.last_gps_stamp = t

        pos = msg.pose.pose.position
        gps_pos = np.array([pos.x, pos.y, pos.z], dtype=float)

        if self.first_gps is None:
            self.first_gps = gps_pos
            self.try_initialise()
            if self.initialised:
                self.latest_stamp = msg.header.stamp
                self.publish(self.latest_stamp)

        if not self.initialised:
            return

        cov = msg.pose.covariance
        if len(cov) == 36 and cov[0] > 0.0 and cov[7] > 0.0 and cov[14] > 0.0:
            R = np.diag([cov[0], cov[7], cov[14]])
        else:
            R = self.R_gps_fallback

        H = np.zeros((3, self.N))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        z = gps_pos
        h = self.x[0:3]

        self.correct(z, h, H, R)

    def on_vo(self, msg):
        t = stamp_to_sec(msg.header.stamp)

        if self.last_vo_stamp > 0.0 and t < self.last_vo_stamp - 0.5:
            self.last_vo_stamp = t
            return
        self.last_vo_stamp = t

        pose = msg.pose.pose
        pos = pose.position
        yaw = quaternion_yaw(pose.orientation)
        vel = msg.twist.twist.linear

        if self.first_vo_yaw is None:
            self.first_vo_yaw = yaw
            self.first_vo_pos = np.array([pos.x, pos.y, pos.z], dtype=float)
            self.try_initialise()

        if not self.initialised:
            return

        # 1. VO Yaw
        H_yaw = np.zeros((1, self.N))
        H_yaw[0, 6] = 1.0
        innov_yaw = np.array([wrap(yaw - self.x[6])])
        S_yaw = H_yaw @ self.P @ H_yaw.T + self.R_vo_yaw
        K_yaw = self.P @ H_yaw.T / S_yaw[0, 0]

        self.x = self.x + (K_yaw * innov_yaw[0]).flatten()
        self.x[6] = wrap(self.x[6])

        I_KH_yaw = np.eye(self.N) - K_yaw @ H_yaw
        self.P = I_KH_yaw @ self.P @ I_KH_yaw.T + K_yaw * self.R_vo_yaw[0, 0] @ K_yaw.T
        self.P = 0.5 * (self.P + self.P.T)
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), 1e-9))

        # 2. VO Body-Velocity
        y_est = self.x[6]
        c, s = math.cos(y_est), math.sin(y_est)
        vx_w, vy_w, vz_w = self.x[3], self.x[4], self.x[5]

        h_vel = np.array([
            c * vx_w + s * vy_w,
            -s * vx_w + c * vy_w,
            vz_w,
        ], dtype=float)

        H_vel = np.zeros((3, self.N))
        H_vel[0, 3] = c
        H_vel[0, 4] = s
        H_vel[1, 3] = -s
        H_vel[1, 4] = c
        H_vel[2, 5] = 1.0
        H_vel[0, 6] = -s * vx_w + c * vy_w
        H_vel[1, 6] = -c * vx_w - s * vy_w
        H_vel[2, 6] = 0.0

        z_vel = np.array([vel.x, vel.y, vel.z], dtype=float)
        self.correct(z_vel, h_vel, H_vel, self.R_vo_vel)

    def publish(self, stamp):
        if not self.initialised:
            return

        if not (np.all(np.isfinite(self.x)) and np.all(np.isfinite(self.P))):
            self.get_logger().error('EKF state contains NaN/Inf! Skipping publication.')
            return

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.position.z = float(self.x[2])
        odom.pose.pose.orientation = yaw_to_quaternion(float(self.x[6]))

        pose_cov = [0.0] * 36
        for i in range(3):
            for j in range(3):
                pose_cov[i * 6 + j] = float(self.P[i, j])
        pose_cov[5 * 6 + 5] = float(self.P[6, 6])
        for i in range(3):
            pose_cov[i * 6 + 5] = float(self.P[i, 6])
            pose_cov[5 * 6 + i] = float(self.P[6, i])
        pose_cov[3 * 6 + 3] = 1e-4
        pose_cov[4 * 6 + 4] = 1e-4
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = float(self.x[3])
        odom.twist.twist.linear.y = float(self.x[4])
        odom.twist.twist.linear.z = float(self.x[5])

        twist_cov = [0.0] * 36
        for i in range(3):
            for j in range(3):
                twist_cov[i * 6 + j] = float(self.P[3 + i, 3 + j])
        twist_cov[5 * 6 + 5] = float(self.Q_rate[6, 6])
        odom.twist.covariance = twist_cov

        self.pub_odom.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()