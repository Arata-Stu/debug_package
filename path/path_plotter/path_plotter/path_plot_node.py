import rclpy
from rclpy.node import Node
from simple_path_interface.msg import PathCoordinates
import matplotlib.pyplot as plt

class PathSubscriber(Node):

    def __init__(self):
        super().__init__('path_subscriber')
        
        # Declare parameters
        self.max_center_points = self.declare_parameter('max_center_points', 10).get_parameter_value().integer_value
        self.max_side_points = self.declare_parameter('max_side_points', 10).get_parameter_value().integer_value
        self.show_center_line = self.declare_parameter('show_center_line', True).get_parameter_value().bool_value
        self.show_side_lines = self.declare_parameter('show_side_lines', True).get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            PathCoordinates,
            'path_coordinates',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Path Subscriber Node has been started with max_center_points: {self.max_center_points}, max_side_points: {self.max_side_points}, show_center_line: {self.show_center_line}, show_side_lines: {self.show_side_lines}')

        # Enable interactive mode
        plt.ion()
        self.figure, self.ax = plt.subplots()
        
        if self.show_center_line:
            self.line_center, = self.ax.plot([], [], 'gray', label='Center Line')
            self.scatter_center = self.ax.scatter([], [], c='green')
        if self.show_side_lines:
            self.line_left, = self.ax.plot([], [], 'blue', label='Left Bound')
            self.scatter_left = self.ax.scatter([], [], c='blue')
            self.line_right, = self.ax.plot([], [], 'red', label='Right Bound')
            self.scatter_right = self.ax.scatter([], [], c='red')
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Path with Lane ID')
        self.ax.legend()

    def pad_array(self, array, length):
        if len(array) >= length:
            return array[:length]
        else:
            return array + [array[-1]] * (length - len(array))

    def listener_callback(self, msg):
        self.get_logger().info('Received Path2DCoordinates message.')

        if self.show_center_line:
            center_x = self.pad_array(msg.center_x, self.max_center_points)
            center_y = self.pad_array(msg.center_y, self.max_center_points)
            # Update plot data for center line
            self.line_center.set_data(center_x, center_y)
            self.scatter_center.set_offsets(list(zip(center_x, center_y)))
        
        if self.show_side_lines:
            left_bound_x = self.pad_array(msg.left_bound_x, self.max_side_points)
            left_bound_y = self.pad_array(msg.left_bound_y, self.max_side_points)
            right_bound_x = self.pad_array(msg.right_bound_x, self.max_side_points)
            right_bound_y = self.pad_array(msg.right_bound_y, self.max_side_points)
            # Update plot data for left bound
            self.line_left.set_data(left_bound_x, left_bound_y)
            self.scatter_left.set_offsets(list(zip(left_bound_x, left_bound_y)))
            # Update plot data for right bound
            self.line_right.set_data(right_bound_x, right_bound_y)
            self.scatter_right.set_offsets(list(zip(right_bound_x, right_bound_y)))

        self.ax.relim()
        self.ax.autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = PathSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
