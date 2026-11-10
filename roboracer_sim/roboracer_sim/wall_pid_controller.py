import math
from statistics import median

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallPidController(Node):
    def __init__(self):
        super().__init__('wall_pid_controller')

        # Parametry regulatora
        self.declare_parameter('kp', 0.6)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('max_steering', 0.6)
        self.declare_parameter('stop_distance', 1.0)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.speed = self.get_parameter('speed').value
        self.max_steering = self.get_parameter('max_steering').value
        self.stop_distance = self.get_parameter('stop_distance').value

        self.scan = None
        self.last_error = None
        self.integral = 0.0
        self.last_time = None

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Wall PID controller uruchomiony.')

    def scan_callback(self, msg):
        self.scan = msg

    def sector_median(self, scan, min_angle, max_angle):
        values = []

        for i, distance in enumerate(scan.ranges):
            angle = math.degrees(
                scan.angle_min + i * scan.angle_increment
            )

            if min_angle <= angle <= max_angle:
                if math.isfinite(distance):
                    if scan.range_min < distance < scan.range_max:
                        values.append(distance)

        if not values:
            return None

        return median(values)

    def publish_stop(self):
        msg = Twist()
        self.publisher.publish(msg)

    def control_loop(self):
        if self.scan is None:
            self.publish_stop()
            return

        now = self.get_clock().now()

        # Zatrzymaj samochód, jeśli dane LiDAR są nieaktualne.
        scan_age = (now.nanoseconds - self.scan.header.stamp.sec * 1_000_000_000
                    - self.scan.header.stamp.nanosec) / 1e9

        if scan_age > 0.5:
            self.get_logger().warning('Brak aktualnego skanu - STOP')
            self.publish_stop()
            return

        left = self.sector_median(self.scan, 75, 105)
        right = self.sector_median(self.scan, -105, -75)
        front = self.sector_median(self.scan, -12, 12)

        if left is None or right is None or front is None:
            self.get_logger().warning('Brak pomiarów - STOP')
            self.publish_stop()
            return

        # Awaryjne zatrzymanie przed przeszkodą.
        if front < self.stop_distance:
            self.publish_stop()
            self.get_logger().warning(
                f'Przeszkoda z przodu: {front:.2f} m - STOP'
            )
            return

        # Dodatkowa ochrona przed zbyt bliską ścianą boczną.
        if min(left, right) < 0.35:
            self.publish_stop()
            self.get_logger().warning('Zbyt bliska ściana - STOP')
            return

        # Dodatni błąd oznacza, że prawa ściana jest bliżej:
        # samochód powinien skręcić w lewo.
        error = left - right

        dt = 0.1
        if self.last_time is not None:
            dt = max((now - self.last_time).nanoseconds / 1e9, 0.001)

        derivative = 0.0
        if self.last_error is not None:
            derivative = (error - self.last_error) / dt

        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))

        steering = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        steering = max(
            -self.max_steering,
            min(self.max_steering, steering)
        )

        # Zwalnianie, gdy przeszkoda jest coraz bliżej.
        speed = self.speed
        if front < 2.0:
            speed *= max(0.0, min(1.0, (front - self.stop_distance) / 1.0))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self.publisher.publish(cmd)

        self.last_error = error
        self.last_time = now

        self.get_logger().info(
            f'L={left:.2f} m R={right:.2f} m '
            f'błąd={error:.3f} skręt={steering:.3f} '
            f'prędkość={speed:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallPidController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
