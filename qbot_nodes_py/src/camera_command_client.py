from qbot_nodes_cpp.srv import CameraCommand
import sys
import rclpy
from rclpy.node import Node


class CameraCommandClient(Node):

    def __init__(self):
        super().__init__('camera_command_client')
        self.cli = self.create_client(CameraCommand, 'uvc_camera/camera_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CameraCommand.Request()

    def send_request(self):
        self.req.command = int(sys.argv[1])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    camera_command_client = CameraCommandClient()
    camera_command_client.send_request()

    rclpy.spin_once(camera_command_client)
    if camera_command_client.future.done():
        try:
            response = camera_command_client.future.result()
        except Exception as e:
            camera_command_client.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            camera_command_client.get_logger().info(
                'Command sent: %d' %
                (camera_command_client.req.command))

    camera_command_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()