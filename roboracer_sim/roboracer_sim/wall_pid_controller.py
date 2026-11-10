import math
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallPidController(Node):
    def __init__(self):
        super().__init__('wall_pid_controller')

        # Deklaracja parametrów regulatora i jazdy
        self.declare_parameter('kp', 0.50)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.8)
        self.declare_parameter('speed', 3.0)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('max_steering', 1.0)
        self.declare_parameter('stop_distance', 0.85)
        self.declare_parameter('side_stop_distance', 0.30)
        self.declare_parameter('lookahead_weight', 0.40)
        self.declare_parameter('d_filter_alpha', 0.30)

        self._load_parameters()
        self.add_on_set_parameters_callback(self._parameters_callback)

        # Zmienne stanu skanu i regulatora PID
        self.scan = None
        self.scan_received_time = None
        self.last_processed_scan_time = None

        self.last_error = None
        self.integral = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0

        # QoS dopasowany do czujników (Best Effort dla LiDAR)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos
        )

        # Wyższa częstotliwość pętli (30 Hz) dla płynniejszej reakcji
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

        self.get_logger().info('Zoptymalizowany Wall PID Controller uruchomiony.')

    def _load_parameters(self):
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.speed = float(self.get_parameter('speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.side_stop_distance = float(self.get_parameter('side_stop_distance').value)
        self.lookahead_weight = float(self.get_parameter('lookahead_weight').value)
        self.d_filter_alpha = float(self.get_parameter('d_filter_alpha').value)

    def _parameters_callback(self, params):
        for param in params:
            if hasattr(self, param.name):
                setattr(self, param.name, float(param.value))
                self.get_logger().info(f'Zaktualizowano parametr {param.name} = {param.value}')
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: LaserScan):
        self.scan = msg
        self.scan_received_time = self.get_clock().now()

    @staticmethod
    def normalize_angle_deg(angle_deg: float) -> float:
        """Normalizuje kąt do przedziału [-180, 180) stopni."""
        return (angle_deg + 180.0) % 360.0 - 180.0

    def extract_sectors(self, scan: LaserScan):
        """
        Jednokrotne przejście po tablicy skanu (O(N)) z obsługą
        wartości inf/max_range (otwarta przestrzeń z przodu).
        """
        sectors = {
            'left': [],
            'right': [],
            'front_left': [],
            'front_right': [],
            'front': []
        }

        max_valid = scan.range_max if math.isfinite(scan.range_max) else 15.0

        for i, distance in enumerate(scan.ranges):
            # Ignoruj wartości NaN oraz pomiary poniżej range_min
            if math.isnan(distance) or distance <= scan.range_min:
                continue

            # Jeśli wiązka nie trafiła w przeszkodę (inf lub > range_max),
            # przypisz zasięg maksymalny zamiast odrzucać pomiar.
            if math.isinf(distance) or distance > max_valid:
                clean_dist = max_valid
            else:
                clean_dist = distance

            raw_angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            angle = self.normalize_angle_deg(raw_angle_deg)

            if 75.0 <= angle <= 105.0:
                sectors['left'].append(clean_dist)
            elif -105.0 <= angle <= -75.0:
                sectors['right'].append(clean_dist)
            elif 35.0 <= angle <= 65.0:
                sectors['front_left'].append(clean_dist)
            elif -65.0 <= angle <= -35.0:
                sectors['front_right'].append(clean_dist)
            elif -12.0 <= angle <= 12.0:
                sectors['front'].append(clean_dist)

        return sectors

    def reset_pid(self):
        self.last_error = None
        self.integral = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0

    def publish_stop(self):
        self.publisher.publish(Twist())
        self.reset_pid()

    def control_loop(self):
        if self.scan is None or self.scan_received_time is None:
            self.publish_stop()
            return

        now = self.get_clock().now()
        scan_age = (now - self.scan_received_time).nanoseconds / 1e9

        # Watchdog: zatrzymaj, jeśli dane z LiDAR-a są starsze niż 0.5 s
        if scan_age > 0.5:
            self.get_logger().warning('Brak aktualnego skanu LiDAR - STOP', throttle_duration_sec=1.0)
            self.publish_stop()
            return

        # Unikaj ponownego przeliczania pochodnej PID dla tej samej ramki skanu
        if self.last_processed_scan_time == self.scan_received_time:
            return
        self.last_processed_scan_time = self.scan_received_time

        sectors = self.extract_sectors(self.scan)

        if any(len(v) == 0 for v in sectors.values()):
            self.get_logger().warning('Puste sektory pomiarowe LiDAR - STOP', throttle_duration_sec=1.0)
            self.publish_stop()
            return

        left = median(sectors['left'])
        right = median(sectors['right'])
        front_left = median(sectors['front_left'])
        front_right = median(sectors['front_right'])
        front = median(sectors['front'])
        front_min = min(sectors['front'])

        # 1. Awaryjne zatrzymanie przed przeszkodą z przodu
        if front_min < self.stop_distance:
            self.publish_stop()
            self.get_logger().warning(
                f'Przeszkoda z przodu ({front_min:.2f} m < {self.stop_distance:.2f} m) - STOP',
                throttle_duration_sec=0.5
            )
            return

        # 2. Zabezpieczenie boczne (zamiast pełnego blokowania ruchu na zawsze,
        # zatrzymujemy tylko poniżej krytycznego progu side_stop_distance)
        if min(left, right) < self.side_stop_distance:
            self.publish_stop()
            self.get_logger().warning(
                f'Zbyt bliska ściana boczna ({min(left, right):.2f} m) - STOP',
                throttle_duration_sec=0.5
            )
            return

        # 3. Obliczenie błędu prowadzenia pośrodku korytarza / toru
        # Dodatni błąd -> więcej miejsca po lewej -> skręć w lewo (dodatnie angular.z)
        side_error = left - right
        lookahead_error = front_left - front_right

        w_look = self.lookahead_weight
        error = (1.0 - w_look) * side_error + w_look * lookahead_error

        # 4. Krok czasowy dt
        dt = 0.05
        if self.last_time is not None:
            dt = max((now - self.last_time).nanoseconds / 1e9, 0.005)

        # 5. Człon różniczkujący (D) z filtrem dolnoprzepustowym EMA
        raw_derivative = 0.0
        if self.last_error is not None:
            raw_derivative = (error - self.last_error) / dt

        alpha = self.d_filter_alpha
        self.filtered_derivative = (
            alpha * raw_derivative + (1.0 - alpha) * self.filtered_derivative
        )

        # 6. Człon całkujący (I) z Anti-Windup oraz zerowaniem przy zmianie znaku błędu
        if self.last_error is not None and (error * self.last_error < 0.0):
            self.integral = 0.0
        else:
            self.integral += error * dt
            self.integral = max(-1.0, min(1.0, self.integral))

        # 7. Wyznaczenie sterowania kątowego
        steering = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_derivative
        )
        steering = max(-self.max_steering, min(self.max_steering, steering))

        # 8. Adaptacyjna prędkość liniowa (zależna od odległości z przodu i kąta skrętu)
        speed = self.speed

        if front < 3.0:
            dist_factor = (front - self.stop_distance) / (3.0 - self.stop_distance)
            dist_factor = max(0.2, min(1.0, dist_factor))
            speed *= dist_factor

        steering_ratio = abs(steering) / max(self.max_steering, 1e-3)
        corner_factor = max(0.35, 1.0 - 0.55 * steering_ratio)
        speed *= corner_factor

        speed = max(self.min_speed, speed)

        # Publikacja komendy ruchu
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steering)
        self.publisher.publish(cmd)

        self.last_error = error
        self.last_time = now

        self.get_logger().info(
            f'L={left:.2f} R={right:.2f} FL={front_left:.2f} FR={front_right:.2f} '
            f'F={front_min:.2f} | err={error:.3f} steer={steering:.3f} v={speed:.2f}',
            throttle_duration_sec=0.25
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()