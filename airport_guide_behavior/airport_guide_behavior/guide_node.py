import rclpy
from rclpy.node import Node


class GuideNode(Node):

    def __init__(self):
        super().__init__('guide_node')
        self.get_logger().info('guide_node iskeleti calisiyor (henuz davranis yok)')


def main(args=None):
    rclpy.init(args=args)
    node = GuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()