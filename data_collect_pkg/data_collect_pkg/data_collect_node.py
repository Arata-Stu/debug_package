import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from ai_object_interface.msg import AIObjectPos
from nav_msgs.msg import Odometry
from simple_path_interface.msg import PathCoordinates
import h5py
import numpy as np
import os

class DataAggregatorNode(Node):
    def __init__(self):
        super().__init__('data_aggregator_node')

        # Subscriptions to the relevant topics
        self.control_subscriber_ = self.create_subscription(
            AckermannControlCommand, 
            '/control/command/control_cmd', 
            self.control_callback, 
            1)

        self.object_subscriber_ = self.create_subscription(
            AIObjectPos, 
            '/debug/filtered_objects', 
            self.object_callback, 
            1)

        self.odometry_subscriber_ = self.create_subscription(
            Odometry, 
            '/localization/kinematic_state', 
            self.odometry_callback, 
            1)

        self.path_subscriber_ = self.create_subscription(
            PathCoordinates, 
            '/debug/path_coordinates', 
            self.path_callback, 
            1)

        # Initialize storage variables
        self.steering_angle_ = None
        self.speed_ = None
        self.acceleration_ = None
        self.brake_position_ = None
        self.x_positions_ = None
        self.y_positions_ = None
        self.odom_position_ = None
        self.odom_orientation_ = None
        self.odom_linear_ = None
        self.odom_angular_ = None
        self.center_x_ = None
        self.center_y_ = None
        self.left_bound_x_ = None
        self.left_bound_y_ = None
        self.right_bound_x_ = None
        self.right_bound_y_ = None

        # Flags to check if data has been received
        self.data_received_flags = {
            'control_received': False,
            'object_received': False,
            'odometry_received': False,
            'path_received': False
        }

        # HDF5 file setup
        self.create_h5_file()

        self.timer_ = None  # Timer will be initialized after receiving all data

    def create_h5_file(self):
        # ディレクトリが存在しない場合は作成
        if not os.path.exists('data'):
            os.makedirs('data')

        # 'data' ディレクトリ内のファイル数をカウント
        existing_files = [f for f in os.listdir('data') if os.path.isfile(os.path.join('data', f))]
        episode_number = len(existing_files) + 1

        # エピソードのファイル名を生成
        self.h5_file_path = f'data/episode{episode_number}.h5'

        # HDF5ファイルを新たに作成
        self.h5_file = h5py.File(self.h5_file_path, 'w')

    def control_callback(self, msg):
        self.steering_angle_ = msg.lateral.steering_tire_angle
        self.speed_ = msg.longitudinal.speed
        self.acceleration_ = msg.longitudinal.acceleration
        self.brake_position_ = msg.longitudinal.jerk  # Using jerk as brake position
        self.data_received_flags['control_received'] = True
        self.get_logger().info(f'Received control data: steering_angle={self.steering_angle_}, speed={self.speed_}, acceleration={self.acceleration_}, brake_position={self.brake_position_}')
        self.check_all_data_received()

    def object_callback(self, msg):
        self.x_positions_ = msg.x
        self.y_positions_ = msg.y
        self.data_received_flags['object_received'] = True
        self.get_logger().info(f'Received object data: x_positions={self.x_positions_}, y_positions={self.y_positions_}')
        self.check_all_data_received()

    def odometry_callback(self, msg):
        self.odom_position_ = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom_orientation_ = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.odom_linear_ = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.odom_angular_ = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
        self.data_received_flags['odometry_received'] = True
        self.get_logger().info(f'Received odometry data: position={self.odom_position_}, orientation={self.odom_orientation_}, linear_velocity={self.odom_linear_}, angular_velocity={self.odom_angular_}')
        self.check_all_data_received()

    def path_callback(self, msg):
        self.center_x_ = msg.center_x
        self.center_y_ = msg.center_y
        self.left_bound_x_ = msg.left_bound_x
        self.left_bound_y_ = msg.left_bound_y
        self.right_bound_x_ = msg.right_bound_x
        self.right_bound_y_ = msg.right_bound_y
        self.data_received_flags['path_received'] = True
        self.get_logger().info(f'Received path data: center_x={self.center_x_}, center_y={self.center_y_}, left_bound_x={self.left_bound_x_}, left_bound_y={self.left_bound_y_}, right_bound_x={self.right_bound_x_}, right_bound_y={self.right_bound_y_}')
        self.check_all_data_received()

    def check_all_data_received(self):
        if all(self.data_received_flags.values()) and self.timer_ is None:
            self.get_logger().info("All data received, starting data aggregation...")
            self.timer_ = self.create_timer(0.1, self.publish_aggregated_data)

    def publish_aggregated_data(self):
        # Get the current time using self.get_clock().now()
        timestep = str(self.get_clock().now().nanoseconds)
        timestep_group = self.h5_file.create_group(timestep)

        # Action group
        action_group = timestep_group.create_group('Action')
        action_group.create_dataset('steering_angle', data=self.steering_angle_)
        action_group.create_dataset('acceleration', data=self.acceleration_)
        action_group.create_dataset('brake_position', data=self.brake_position_)

        # Observation group
        observation_group = timestep_group.create_group('Observation')
        observation_group.create_dataset('speed', data=self.speed_)
        
        if self.x_positions_ is not None:
            observation_group.create_dataset('x_positions', data=np.array(self.x_positions_))
            observation_group.create_dataset('y_positions', data=np.array(self.y_positions_))

        if self.odom_position_ is not None:
            observation_group.create_dataset('odom_position', data=self.odom_position_)
            observation_group.create_dataset('odom_orientation', data=self.odom_orientation_)
            observation_group.create_dataset('odom_linear', data=self.odom_linear_)
            observation_group.create_dataset('odom_angular', data=self.odom_angular_)

        if self.center_x_ is not None:
            observation_group.create_dataset('center_x', data=np.array(self.center_x_))
            observation_group.create_dataset('center_y', data=np.array(self.center_y_))
            observation_group.create_dataset('left_bound_x', data=np.array(self.left_bound_x_))
            observation_group.create_dataset('left_bound_y', data=np.array(self.left_bound_y_))
            observation_group.create_dataset('right_bound_x', data=np.array(self.right_bound_x_))
            observation_group.create_dataset('right_bound_y', data=np.array(self.right_bound_y_))

def main(args=None):
    rclpy.init(args=args)
    node = DataAggregatorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    node.h5_file.close()

if __name__ == '__main__':
    main()
