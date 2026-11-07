import rclpy
from rclpy.node import Node


class AlgilamaNode(Node):

    def __init__(self):
        super().__init__('algilama_node')
        self.get_logger().info('algilama_node iskeleti calisiyor (henuz algilama yok)')


def main(args=None):
    rclpy.init(args=args)
    node = AlgilamaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()