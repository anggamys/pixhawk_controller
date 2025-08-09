import sys
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

try:
    from transformations import euler_from_quaternion
except ImportError:
    from scipy.spatial.transform import Rotation as R
    def euler_from_quaternion(quat):
        return R.from_quat(quat).as_euler('xyz', degrees=False)


class ImuLoggerNode(Node):
    def __init__(self):
        super().__init__('imu_logger_node')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        try:
            roll, pitch, yaw = euler_from_quaternion(quaternion)
        except Exception as e:
            # Jangan spam warning tiap frame; log sesekali kalau perlu
            self.get_logger().warning(f"Gagal konversi quaternion: {e}")
            roll, pitch, yaw = 0.0, 0.0, 0.0

        ax, ay, az = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )

        display = (
            f"ROLL: {roll:.2f}  PITCH: {pitch:.2f}  YAW: {yaw:.2f}  "
            f"AX: {ax:.2f}  AY: {ay:.2f}  AZ: {az:.2f}"
        )
        # timpa satu baris di terminal
        sys.stdout.write('\r\033[K' + display)
        sys.stdout.flush()


def _handle_sigterm(signum, frame):
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


def main(args=None):
    signal.signal(signal.SIGTERM, _handle_sigterm)

    rclpy.init(args=args)
    node = ImuLoggerNode()

    try:
        rclpy.spin(node)  # Ctrl-C -> KeyboardInterrupt
    except KeyboardInterrupt:
        sys.stdout.write('\n[IMU Logger stopped]\n')
        sys.stdout.flush()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
