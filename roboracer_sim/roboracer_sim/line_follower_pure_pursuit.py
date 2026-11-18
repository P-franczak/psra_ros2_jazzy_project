import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PointStamped


class LineFollowerPurePursuit(Node):
    """
    Line follower: camera -> line pixels -> ground-plane points (robot frame)
    -> Pure Pursuit lookahead point -> curvature -> Twist(v, w = v * kappa).
    """

    DEFAULTS = {
        # Pure Pursuit / jazda
        'speed': 0.8,
        'min_speed': 0.5,
        'lookahead_gain': 0.8,       # Ld = gain * v
        'lookahead_min': 0.8,        # [m] (najblizsza widoczna ziemia ~0.63 m od tylnej osi)
        'lookahead_max': 2.0,        # [m]
        'curve_speed_gain': 0.6,     # v = speed / (1 + gain * |kappa|)
        'wheelbase': 0.325,          # [m] z URDF
        'steering_limit': 0.42,      # [rad] z URDF -> kappa_max = tan(limit)/wheelbase
        'lost_timeout': 0.3,         # [s] tyle trzymaj ostatnia komende po utracie linii
        'recovery_timeout': 4.0,     # [s] potem szukaj linii; po tym czasie STOP
        'recovery_speed': 0.3,       # [m/s] predkosc podczas szukania
        'recovery_curvature': 0.8,   # [1/m] luk w strone, w ktora linia uciekla

        # Detekcja linii (kolor w RGB)
        # track.sdf/model.sdf: centerline = zolta (1, 0.85, 0.05) -> ~ (255, 217, 13), po oswietleniu nieco inaczej.
        # NIE biala: krawedzie toru i bariery sa biale, droga szara, bariery czerwone.
        'line_rgb': [250, 215, 20],
        'color_tolerance': 70,
        'min_line_pixels': 2,        # cienka linia: daleko to tylko kilka pikseli
        'roi_top_frac': 0.55,        # ignoruj gorna czesc obrazu (horyzont ~ 0.5 przy pitch=0)
        'n_scanlines': 24,
        'min_points': 3,
        'max_point_dist': 3.0,       # [m] (dalej rozdzielczosc ziemi jest zbyt slaba)

        # Geometria kamery wzgledem punktu odniesienia pojazdu
        # Z URDF: mount (0.395, 0, 0.0575) od base_link + 0.0125 do optyki; base_link 0.05 m nad ziemia
        'camera_height': 0.12,       # [m] nad ziemia
        'camera_pitch_deg': 0.0,     # [deg] w dol (URDF: rpy = 0 0 0)
        'camera_x_offset': 0.395,    # [m] do przodu od tylnej osi (base_link)
        'camera_y_offset': 0.0,      # [m] w lewo

        'base_frame': 'base_footprint',
    }

    def __init__(self):
        super().__init__('line_follower_pure_pursuit')

        for name, default in self.DEFAULTS.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)
        self.add_on_set_parameters_callback(self._parameters_callback)

        self.K = None                 # (fx, fy, cx, cy)
        self.track_u = None           # kolumna sledzonej linii (ciaglosc)
        self.last_cmd = Twist()
        self.last_valid_time = None
        self.last_v = float(self.speed)
        self.last_far_y = 0.0         # bok (y) najdalszego widzianego punktu linii
        self.recovering = False

        self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(
            PointStamped, '/line_follower/target', 10)
        # Maska debugowa (publikowana tylko gdy ktos subskrybuje)
        self.mask_pub = self.create_publisher(Image, '/line_follower/mask', 1)

        # Jedno miejsce publikacji komend + watchdog
        self.create_timer(1.0 / 30.0, self.publish_loop)

        self.get_logger().info('Line follower (Pure Pursuit) uruchomiony.')

    # ------------------------------------------------------------------ params
    def _parameters_callback(self, params):
        for p in params:
            if p.name in self.DEFAULTS:
                setattr(self, p.name, p.value)
                self.get_logger().info(f'Zaktualizowano {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # --------------------------------------------------------------- callbacks
    def info_callback(self, msg: CameraInfo):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy

    def image_callback(self, msg: Image):
        if self.K is None:
            self.get_logger().warning(
                'Brak camera_info - czekam', throttle_duration_sec=1.0)
            return

        img = self.decode_image(msg)
        if img is None:
            return

        pts = self.extract_line_points(img, msg.header)
        if len(pts) < self.min_points:
            self.get_logger().warning(
                'Nie widze linii', throttle_duration_sec=1.0)
            return  # zostaje last_cmd do czasu lost_timeout

        # Pure Pursuit
        ld = self.lookahead_gain * self.last_v
        ld = max(self.lookahead_min, min(self.lookahead_max, ld))
        target, ld_eff = self.pick_target(pts, ld)

        kappa = 2.0 * target[1] / (ld_eff ** 2)
        kappa_max = math.tan(self.steering_limit) / self.wheelbase
        kappa = max(-kappa_max, min(kappa_max, kappa))

        v = self.speed / (1.0 + self.curve_speed_gain * abs(kappa))
        v *= max(0.5, min(1.0, ld_eff / ld))   # widoczna linia krotsza niz Ld -> zwolnij
        v = max(self.min_speed, min(self.speed, v))
        w = v * kappa

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.last_cmd = cmd
        self.last_v = float(v)
        self.last_far_y = float(pts[-1][1])
        self.last_valid_time = self.get_clock().now()

        tp = PointStamped()
        tp.header.stamp = msg.header.stamp
        tp.header.frame_id = self.base_frame
        tp.point.x, tp.point.y = float(target[0]), float(target[1])
        self.target_pub.publish(tp)

        self.get_logger().info(
            f'Ld={ld_eff:.2f} target=({target[0]:.2f},{target[1]:.2f}) '
            f'kappa={kappa:.3f} v={v:.2f} w={w:.2f}',
            throttle_duration_sec=0.25)

    def publish_loop(self):
        if self.last_valid_time is None:
            self.cmd_pub.publish(Twist())
            return
        age = (self.get_clock().now() - self.last_valid_time).nanoseconds / 1e9

        if age <= self.lost_timeout:
            self.recovering = False
            self.cmd_pub.publish(self.last_cmd)
        elif age <= self.lost_timeout + self.recovery_timeout:
            # Szukanie linii: wolno, lukiem w strone, w ktora linia uciekla
            side = 1.0 if self.last_far_y >= 0.0 else -1.0
            if not self.recovering:
                self.recovering = True
                self.track_u = None
                self.get_logger().warning(
                    f'Linia zgubiona (ostatni punkt y={self.last_far_y:+.2f} m) '
                    f'- szukam w {"lewo" if side > 0 else "prawo"}')
            cmd = Twist()
            cmd.linear.x = float(self.recovery_speed)
            cmd.angular.z = float(side * self.recovery_curvature * self.recovery_speed)
            self.cmd_pub.publish(cmd)
        else:
            self.cmd_pub.publish(Twist())
            self.last_v = float(self.speed)
            self.track_u = None

    # ------------------------------------------------------------- perception
    def decode_image(self, msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warning(
                f'Nieobslugiwane kodowanie: {msg.encoding}',
                throttle_duration_sec=2.0)
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        img = data.reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            img = img[:, :, ::-1]
        return img

    def publish_mask(self, mask, v0, h, w, header):
        full = np.zeros((h, w), dtype=np.uint8)
        full[v0:] = mask.astype(np.uint8) * 255
        out = Image()
        out.header = header
        out.height, out.width = h, w
        out.encoding = 'mono8'
        out.step = w
        out.data = full.tobytes()
        self.mask_pub.publish(out)

    def extract_line_points(self, img, header):
        """Zwraca liste punktow (x_forward, y_left) w metrach, od najblizszego."""
        h, w, _ = img.shape
        v0 = int(h * self.roi_top_frac)

        target = np.array(self.line_rgb, dtype=np.int16)
        diff = np.abs(img[v0:].astype(np.int16) - target).max(axis=2)
        mask = diff <= self.color_tolerance

        if self.mask_pub.get_subscription_count() > 0:
            self.publish_mask(mask, v0, h, w, header)

        fx, fy, cx, cy = self.K
        pitch = math.radians(self.camera_pitch_deg)
        sp, cp = math.sin(pitch), math.cos(pitch)

        rows = np.linspace(h - 1, v0, self.n_scanlines).astype(int)
        track_u = self.track_u if self.track_u is not None else w / 2.0

        pts = []
        for r in rows:
            cols = np.flatnonzero(mask[r - v0])
            if len(cols) < self.min_line_pixels:
                continue

            runs = np.split(cols, np.where(np.diff(cols) > 1)[0] + 1)
            runs = [run for run in runs if len(run) >= self.min_line_pixels]
            if not runs:
                continue

            # wybierz fragment najblizszy poprzednio sledzonej pozycji
            centers = np.array([run.mean() for run in runs])
            u = float(centers[np.argmin(np.abs(centers - track_u))])
            track_u = u

            # piksel -> promien -> przeciecie z plaszczyzna ziemi
            xn = (u - cx) / fx
            yn = (float(r) - cy) / fy
            fwd = cp - yn * sp
            down = sp + yn * cp
            if down <= 1e-3:
                continue
            t = self.camera_height / down
            X = t * fwd + self.camera_x_offset
            Y = -t * xn + self.camera_y_offset
            if X <= 0.0 or math.hypot(X, Y) > self.max_point_dist:
                continue
            pts.append((X, Y))

        if pts:
            self.track_u = track_u
        return pts

    # ----------------------------------------------------------- pure pursuit
    @staticmethod
    def pick_target(pts, ld):
        """Punkt sciezki w odleglosci ld (interpolacja); zwraca (punkt, ld_eff)."""
        p = np.array(pts)
        d = np.hypot(p[:, 0], p[:, 1])

        ahead = np.flatnonzero(d >= ld)
        if len(ahead) == 0:                 # linia krotsza niz Ld -> najdalszy punkt
            return p[-1], max(d[-1], 1e-3)
        i = int(ahead[0])
        if i == 0:                          # linia zaczyna sie za Ld
            return p[0], max(d[0], 1e-3)

        s = (ld - d[i - 1]) / max(d[i] - d[i - 1], 1e-6)
        target = p[i - 1] + s * (p[i] - p[i - 1])
        return target, ld


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()