import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Set up the serial connection
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Update the port as needed
        time.sleep(2)  # Allow time for Arduino reset

        # Subscriber to listen for messages
        self.subscription = self.create_subscription(
            Int32,
            'led_control',
            self.listener_callback,
            10
        )
        self.get_logger().info('SerialNode initialized and listening on /led_control')

    def listener_callback(self, msg):
        pin = msg.data
        self.get_logger().info(f'Sending pin number: {pin} to Arduino')
        try:
            self.serial_port.write(f'{pin}\n'.encode())  # Send pin number to Arduino
            response = self.serial_port.readline().decode().strip()  # Read Arduino response
            self.get_logger().info(f'Arduino responded: {response}')
        except Exception as e:
            self.get_logger().error(f'Error communicating with Arduino: {e}')

    def destroy_node(self):
        # Close the serial port when the node is destroyed
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
