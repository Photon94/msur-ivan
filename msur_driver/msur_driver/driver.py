import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from msur_stm_driver.service import Client
from msur_stm_driver.protocol import XThrust, YThrust, ZThrust


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
        self.client = Client()
        self.get_logger().info('Узел драйвера запущен!')
    
    def driver_callback(self, msg: Twist):
        self.get_logger().info(f"{msg}")
        self.client.send([XThrust(value=msg.linear.x), YThrust(value=msg.linear.y), ZThrust(value=msg.linear.z)])


def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
