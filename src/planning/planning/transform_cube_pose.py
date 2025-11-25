import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R
import numpy as np



class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose_base', 10)

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        if self.cube_pose is None:
            self.cube_pose = self.transform_cube_pose(msg)
            self.cube_pose_pub.publish(self.transform_cube_pose(msg))
            self.get_logger().info('Received cube pose')

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            Point: point in base_link_frame in form [x, y, z]
        """

        target_frame = 'base_link'
        source_frame = msg.header.frame_id

        try:
            tf_msg = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'lookup_transform failed: {e}')
            return None
        
        #Translation
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        tz = tf_msg.transform.translation.z

        #Rotation
        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w

        rot = R.from_quat([qx, qy, qz, qw]).as_matrix()

        original_point = np.array([msg.point.x, msg.point.y, msg.point.z])

        #Point from original frame to base_link frame
        point_base_link = rot @ original_point + np.array([tx, ty, tz])

        out_msg = PointStamped()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = target_frame
        out_msg.point.x = float(point_base_link[0])
        out_msg.point.y = float(point_base_link[1])
        out_msg.point.z = float(point_base_link[2])


        return out_msg

def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
