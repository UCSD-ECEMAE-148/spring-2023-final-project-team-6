import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray, String
from geometry_msgs.msg import Twist
import time
import os

from sensor_msgs.msg import LaserScan
import math

NODE_NAME = 'lane_guidance_node'
CENTROID_TOPIC_NAME = '/centroid'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
SUBSCRIBER_TOPIC_NAME = '/scan'
LD_NODE_NAME = 'lane_detection_node'


class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.sub = self.create_subscription(LaserScan, SUBSCRIBER_TOPIC_NAME, self.detect_obstacle, 10)
        self.sub #prevent unused variable warning
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.centroid_subscriber = self.create_subscription(Float32, CENTROID_TOPIC_NAME, self.controller, 10)
        self.centroid_subscriber
        
        
        
        self.subscription = self.create_subscription(
            String,
            LD_NODE_NAME,
            self.controller,
            10)
        self.subscription 
        
       
        self.publisher_ = self.create_publisher(Float32, NODE_NAME , 10)
        self.i = 0.0
        
       
        self.obstacle_detected = 0.0
        self.contours_silver = 0.0
        self.obstacle_ranges = 0.0
        
       
       
        self.viewing_angle = 360
        self.max_distance_tolerance = 0.4
        self.min_distance_tolerance = 0.1
        self.min_angle = 0
        self.min_distance = 0
        
        

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0)
            ])
        self.Kp = self.get_parameter('Kp_steering').value # between [0,1]
        self.Ki = self.get_parameter('Ki_steering').value # between [0,1]
        self.Kd = self.get_parameter('Kd_steering').value # between [0,1]
        self.error_threshold = self.get_parameter('error_threshold').value # between [0,1]
        self.zero_throttle = self.get_parameter('zero_throttle').value # between [-1,1] but should be around 0
        self.max_throttle = self.get_parameter('max_throttle').value # between [-1,1]
        self.min_throttle = self.get_parameter('min_throttle').value # between [-1,1]
        self.max_right_steering = self.get_parameter('max_right_steering').value # between [-1,1]
        self.max_left_steering = self.get_parameter('max_left_steering').value # between [-1,1]

       
        self.Ts = float(1/20)
        self.ek = 0 
        self.ek_1 = 0 
        self.proportional_error = 0
        self.derivative_error = 0
        self.integral_error = 0
        self.integral_max = 1E-8
        
        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )

    
    def detect_obstacle(self, data):
       def detect_obstacle(self, data):
    self.get_logger().info('Detect_obstacle')
    self.obstacle_ranges = data.ranges[-1]
    total_number_of_scans = len(data.ranges)
    scans_per_degree = total_number_of_scans / self.viewing_angle
    angle_values = list(range(80, 101, 2))
    self.min_distance, self.min_angle = self.calculate_min_distance_and_angle(angle_values, scans_per_degree, data)
    self.get_logger().info(f'Min Distance {str(self.min_distance)} at {str(self.min_angle)} degrees.')
    self.evaluate_obstacle(self.min_distance, self.min_angle)

    def calculate_min_distance_and_angle(self, angle_values, scans_per_degree, data):
        range_values = []
        for angle in angle_values:
            bs = data.ranges[round(angle * scans_per_degree)]
            if self.max_distance_tolerance >= bs >= self.min_distance_tolerance:
                range_values.append(bs)
            else:
                range_values.append(self.max_distance_tolerance + 1)
        min_distance = min(range_values)
        min_angle_index = range_values.index(min_distance)
        min_angle = angle_values[min_angle_index]
        return min_distance, min_angle

    def evaluate_obstacle(self, min_distance, min_angle):
        if self.max_distance_tolerance >= abs(min_distance) >= self.min_distance_tolerance:
            if min_angle > 180:
                min_angle -= 360
            angle_rad = math.radians(min_angle)
            normalized_angle = math.sin(angle_rad)
            self.obstacle_detected = 1.0
        else:
            min_distance = -1.0
            normalized_angle = -1.0
            self.obstacle_detected = 0.0
        self.publish_obstacle_detected()

    def publish_obstacle_detected(self):
        obst = Float32()
        obst.data = float(self.i)
        self.publisher_.publish(obst)
        obstacle_string = str(obst.data)
        self.i = self.obstacle_ranges

        
        
    def controller(self, data):
        # setting up PID control
        self.ek = data.data

        # Throttle gain scheduling (function of error)
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Steering PID terms
        self.proportional_error = self.Kp * self.ek
        self.derivative_error = self.Kd * (self.ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * self.ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

        # Publish values
        try:
            if self.obstacle_detected == 1.0:
                self.twist_cmd.angular.z = 0.0
                self.twist_cmd.linear.x = 0.0
                self.twist_publisher.publish(self.twist_cmd)
            else:
                # publish control signals
                self.twist_cmd.angular.z = steering_float
                self.twist_cmd.linear.x = throttle_float
                self.twist_publisher.publish(self.twist_cmd)

                # shift current time and error values to previous values
                self.ek_1 = self.ek

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)
            
    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 


def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
