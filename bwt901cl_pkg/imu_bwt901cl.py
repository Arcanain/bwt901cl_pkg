import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from .src.bwt901cl import BWT901CL

class Imu901cl(Node):
    def __init__(self, time_interval=1.0):
        super().__init__('imu_bwt901cl')
        
        # Create a TransformBroadcaster to broadcast IMU's pose
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a publisher for IMU raw data
        self.imu_publisher = self.create_publisher(Imu, 'imu_data_raw', 10)
        
        # Timer for publishing data
        self.tmr = self.create_timer(time_interval, self.timer_callback)
        
        # Initialize IMU sensor
        self.imu_sensor = BWT901CL("/dev/ttyUSB1")

    def timer_callback(self):
        # Get data from IMU
        angle, angular_velocity, accel, temp, magnetic, quaternion, time = self.imu_sensor.getData()
        
        # Assume angle is a list of Euler angles in degrees: [roll, pitch, yaw]
        # Convert Euler angles to quaternion
        r = R.from_euler('xyz', angle, degrees=True)  # Convert from degrees to quaternion
        quaternion = r.as_quat()  # Returns [x, y, z, w]

        # Create a TransformStamped message for broadcasting IMU pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # The fixed parent frame
        t.child_frame_id = 'imu_link'    # The IMU frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

        # Publish the raw IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Fill orientation with quaternion (if available)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        # Fill angular velocity (assuming angular_velocity is a list [x, y, z])
        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]

        # Fill linear acceleration (assuming accel is a list [x, y, z])
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        # Publish the IMU raw data message
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node_imu_bwt901cl = Imu901cl(time_interval=0.1)
    rclpy.spin(node_imu_bwt901cl)

    node_imu_bwt901cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
