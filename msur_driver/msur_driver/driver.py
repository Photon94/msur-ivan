import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from msur_packages.driver.service import Client
from msur_packages.driver.protocol import XThrust, YThrust, ZThrust, Yaw, PidStats
from msur_msgs.msg import PidStatus


class Driver(Node):
    """
    Класс слушает топик cmd_vel
    После уничтожения ноды необходимо обнулить скорости аппарата!
    """
    header = Header()
    header.frame_id = 'driver'

    def __init__(self):
        super().__init__(self.header.frame_id)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.driver_callback, 10)
        self.pid_subscription = self.create_subscription(Twist, 'control/pid', self.pid_callback, 10)
        self.client = Client()
        self.get_logger().info('Узел драйвера запущен!')

    def driver_callback(self, msg: Twist):
        self.get_logger().info(f"{msg}")
        self.client.send([XThrust(value=msg.linear.x), YThrust(value=msg.linear.y), ZThrust(value=msg.linear.z), Yaw(value=msg.angular.z)])


    def pid_callback(self, msg: PidStatus):
        self.get_logger().info(f"{msg}")
        self.client.send([PidStats(yaw=msg.yaw, pitch=msg.pitch, roll=msg.roll), ])


def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
