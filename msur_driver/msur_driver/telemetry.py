from rclpy.node import Node
import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import BatteryState, Imu
from msur_msgs.msg import PayloadConfig, ModulesError, PidStatus, Leaks
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from msur_packages.driver import service


class Telemetry(Node):
    """
    Класс предоставляет телеметрию в топики
    """
    header = Header()
    header.frame_id = 'telemetry'

    def __init__(self) -> None:
        super().__init__(self.header.frame_id)
        self.payload_publisher = self.create_publisher(PayloadConfig, 'telemetry/payload', 10)
        self.leaks_publisher = self.create_publisher(Leaks, 'telemetry/leaks', 10)
        self.pid_publisher = self.create_publisher(PidStatus, 'telemetry/pid', 10)
        self.errors_publisher = self.create_publisher(ModulesError, 'telemetry/errors', 10)
        self.battery_publisher = self.create_publisher(BatteryState, 'telemetry/battery', 10)
        self.imu_publisher = self.create_publisher(Imu, 'telemetry/orientation', 10)
        self.client = service.Client()
        self.timer = self.create_timer(0.1, self.callback)
        self.get_logger().info('Узел публикации телеметрии запущен!')

    def callback(self):
        # Устанавливаем тайм-штамп
        self.header.stamp = self.get_clock().now().to_msg()

        telemetry = self.client.telemetry()
        if telemetry is None:
            self.get_logger().warning('Телеметрия не доступна, проверьте промежуточный сервис')
        # Публикуем статус протечек
        self.leaks_publisher.publish(Leaks(header=self.header, main=telemetry.leak.main, imu=telemetry.leak.imu))
        # Публикуем статус ошибок модулей
        self.errors_publisher.publish(ModulesError(header=self.header, pressure_sensor=telemetry.device_error.pressure_sensor, imu_module=telemetry.device_error.imu))
        # Публикуем статус пид регуляторов
        self.pid_publisher.publish(PidStatus(**telemetry.pid_stat.dict(), header=self.header))
        # Публикуем состояние полезной нагрузки
        self.payload_publisher.publish(PayloadConfig(header=self.header, magnet_1=telemetry.devices_stat.em_1, magnet_2=telemetry.devices_stat.em_2))
        # Публикуем состояние IMU
        self.imu_publisher.publish(Imu(header=self.header, angular_velocity=telemetry.get_vector(Vector3)))
        # Публикуем состояние батареи
        self.battery_publisher.publish(BatteryState(voltage=telemetry.voltage, current=telemetry.current, present=True))


def main(args=None):
    rclpy.init(args=args)
    node = Telemetry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
