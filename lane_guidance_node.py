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
        
        
         #My personal attempt at subscribing to data from the calibration node 
        self.subscription = self.create_subscription(
            String,
            LD_NODE_NAME,
            self.controller,
            10)
        self.subscription  # prevent unused variable warning
        
        #My personal attempt at publishing a value to lane guidance
        self.publisher_ = self.create_publisher(Float32, NODE_NAME , 10)
        self.i = 0.0
        
        #for obstacle detection function
        self.obstacle_detected = 0.0
        self.contours_silver = 0.0
        self.obstacle_ranges = 0.0
        
        #My personal attempt to subscribe to the LiDAR
       
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

        # initializing PID control
        self.Ts = float(1/20)
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
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

     #Here is a funciton I added for obstacle detection
    def detect_obstacle(self, data):
        self.get_logger().info(f'Detect_obstacle')
        #CODE THAT I PUT IN 
        self.obstacle_ranges = data.ranges[len(data.ranges) - 1]
        #END OF CODE THAT I PUT IN 
        
        total_number_of_scans = len(data.ranges)
        scans_per_degree = float(total_number_of_scans/self.viewing_angle)
        #angle_values = [1,5,10,15,20,25,30,35,40,45,50,55,60,65,355,350,345,340,335,330,325,320,315,310,305,300,295]
        angle_values = [80,82,84,86,88,90,92,94,96,98,100]
        range_values = []
        for angle in angle_values:
            bs = data.ranges[round(angle*scans_per_degree)]
            if self.max_distance_tolerance >= bs >= self.min_distance_tolerance:
                range_values.append(data.ranges[round(angle*scans_per_degree)])
            else:
                range_values.append(float(self.max_distance_tolerance)+1)
        self.min_distance = min(range_values)
        min_angle_index = range_values.index(min(range_values))
        self.min_angle = angle_values[min_angle_index]
        self.get_logger().info(f'Min Distance {str(self.min_distance)} at {str(self.min_angle)} degrees.')
 
        obstacle_info = []
        if self.max_distance_tolerance >= abs(self.min_distance) >= self.min_distance_tolerance:
            #make the angle nuegative if its on the right side
            if self.min_angle > 180: self.min_angle = self.min_angle - 360
            angle_rad = (self.min_angle * math.pi) / 180
            normalized_angle = math.sin(angle_rad)
            self.obstacle_detected = 1.0
        else:
            # nonsense values
            self.min_distance = -1.0
            normalized_angle = -1.0
            self.obstacle_detected = 0.0
            
        #My personal attampt at publishing the obstacle_detected value 
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
            #     #We are noe entering the part of the coder where we manuver around an object, these values can be tweaked
            #     if self.min_angle > 0:
            #         #Cone is to the left
            #         if self.min_angle < 60:
            #             #This function is to make the car turn to the left side to begin moving around the cone
            #             self.twist_cmd.angular.z = 0.4
            #             self.twist_cmd.linear.x = 0.1
            #             self.twist_publisher.publish(self.twist_cmd)
            #         else:
            #             #By now the car should be facing off to the left and is ready to make a curved path around the cone 
            #             self.twist_cmd.linear.x = 0.4 
            #             self.twist_cmd.angular.z = 0.2
            #             self.twist_publisher.publish(self.twist_cmd)
            #     else:
            #         #Cone is strait ahead or to the right
            #         if self.contours_silver > 3:
            #             #The white line is dotted and we can go to the right 
            #             abs_min_angle = abs(self.min_angle) #This is to take into account that the angle will be negative if the object is to the left
            #             if abs_min_angle < 60:
            #                 #The car will start turning to the right 
            #                 self.twist_cmd.angular.z = -0.4
            #                 self.twist_cmd.linear.x = 0.1 
            #                 self.twist_publisher.publish(self.twist_cmd)
            #             else:
            #                 #Now the car is turned to the right side, it is time for it to make a curved path around the cone 
            #                 self.twist_cmd.linear.x = 0.4 
            #                 self.twist_cmd.angular.z = -0.2
            #                 self.twist_publisher.publish(self.twist_cmd)
            #         else:
            #             #The white line is solid and we need to go to the left 
            #             if self.min_angle < 60:
            #                 #This function is to make the car turn to the left side to begin moving around the cone
            #                 self.twist_cmd.angular.z = 0.4
            #                 self.twist_cmd.linear.x = 0.1
            #                 self.twist_publisher.publish(self.twist_cmd)
            #             else:
            #                 #By now the car should be facing off to the left and is ready to make a curved path around the cone 
            #                 self.twist_cmd.linear.x = 0.4 
            #                 self.twist_cmd.angular.z = 0.2
            #                 self.twist_publisher.publish(self.twist_cmd)
            # #Obstacle not detected, we can go forth with the follow the yellow line sequence
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
