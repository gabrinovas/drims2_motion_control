import rclpy
from rclpy.node import Node
from drims2_msgs.srv import DiceIdentification
from geometry_msgs.msg import PoseStamped
import random
from rclpy.qos import QoSProfile

class FakeDiceIdentificationServer(Node):

    def __init__(self):
        super().__init__('fake_dice_identification_server')
        
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('random_face', False)
        self.declare_parameter('face', 1)

        self.declare_parameter('position.x', 1.0)
        self.declare_parameter('position.y', 2.0)
        self.declare_parameter('position.z', 3.0)

        self.declare_parameter('orientation.x', 0.0)
        self.declare_parameter('orientation.y', 0.0)
        self.declare_parameter('orientation.z', 0.0)
        self.declare_parameter('orientation.w', 1.0)

        self.srv = self.create_service(
            DiceIdentification,
            'dice_identification',
            self.dice_identification_callback,
            qos_profile=QoSProfile(depth=10)
        )
        self.get_logger().info('Fake DiceIdentification service ready.')

    def dice_identification_callback(self, request, response):

        random_face = self.get_parameter('random_face').get_parameter_value().bool_value
        if random_face:
            face = random.randint(1, 6)
        else:
            face = self.get_parameter('face').get_parameter_value().integer_value
            
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        pose.pose.position.x = self.get_parameter('position.x').get_parameter_value().double_value
        pose.pose.position.y = self.get_parameter('position.y').get_parameter_value().double_value
        pose.pose.position.z = self.get_parameter('position.z').get_parameter_value().double_value

        pose.pose.orientation.x = self.get_parameter('orientation.x').get_parameter_value().double_value
        pose.pose.orientation.y = self.get_parameter('orientation.y').get_parameter_value().double_value
        pose.pose.orientation.z = self.get_parameter('orientation.z').get_parameter_value().double_value
        pose.pose.orientation.w = self.get_parameter('orientation.w').get_parameter_value().double_value

        response.face_number = face
        response.pose = pose
        response.success = True

        self.get_logger().info(f'Returning face: {face}, pose: [{pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}]')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakeDiceIdentificationServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
