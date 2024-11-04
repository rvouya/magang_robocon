import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleDrawing(Node):
    def __init__(self):
        super().__init__('turtle_drawing')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def rotate_and_draw(self, angle, duration, line_length):
        msg = Twist()
        
        # Rotasi turtle ke sudut yang diinginkan
        msg.angular.z = math.radians(angle)
        self.publisher.publish(msg)
        self.get_logger().info(f'Berputar {angle} derajat.')
        time.sleep(duration)

        # Hentikan rotasi
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        time.sleep(1)

        # Maju untuk menggambar garis
        if line_length > 0:
            msg.linear.x = 1.0
            self.publisher.publish(msg)
            self.get_logger().info(f'Maju untuk menggambar garis sepanjang {line_length} meter.')
            time.sleep(line_length)

            # Hentikan gerakan setelah menggambar
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            time.sleep(1)

        # Reset sudut kembali ke nol
        msg.angular.z = -math.radians(angle)  # Rotasi berlawanan untuk reset
        self.publisher.publish(msg)
        self.get_logger().info(f'Mengatur ulang sudut ke 0 derajat.')
        time.sleep(duration)

        # Hentikan rotasi setelah reset
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    turtle_drawing = TurtleDrawing()

    # Contoh pemanggilan fungsi
    turtle_drawing.rotate_and_draw(angle=-90, duration=2, line_length=2.5)
    turtle_drawing.rotate_and_draw(angle=0, duration=2, line_length=5.0)
    turtle_drawing.rotate_and_draw(angle=90, duration=2, line_length=5.0)
    turtle_drawing.rotate_and_draw(angle=180, duration=2, line_length=5.0)

    rclpy.spin(turtle_drawing)
    turtle_drawing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
