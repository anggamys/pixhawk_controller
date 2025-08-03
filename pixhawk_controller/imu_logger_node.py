import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

try:
    from transformations import euler_from_quaternion
except ImportError:
    from scipy.spatial.transform import Rotation as R
    def euler_from_quaternion(quat):
        return R.from_quat(quat).as_euler('xyz', degrees=False)

import sys

class ImuLoggerNode(Node):
    def __init__(self):
        super().__init__('imu_logger_node')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos
        )

    def imu_callback(self, msg):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        try:
            roll, pitch, yaw = euler_from_quaternion(quaternion)
        except Exception as e:
            self.get_logger().warning(f"Gagal konversi quaternion: {e}")
            roll, pitch, yaw = 0.0, 0.0, 0.0
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        display = (
            f"ROLL: {roll:.2f}  PITCH: {pitch:.2f}  YAW: {yaw:.2f}  "
            f"AX: {ax:.2f}  AY: {ay:.2f}  AZ: {az:.2f}"
        )

        sys.stdout.write('\r\033[K' + display)
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = ImuLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[IMU Logger stopped]')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
