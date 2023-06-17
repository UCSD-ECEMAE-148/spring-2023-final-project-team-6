#New Container OpenCVTests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from sensor_msgs.msg import LaserScan

#Added Code
from matplotlib import pyplot as plt
import imutils
import time

# Nodes in this program
NODE_NAME = 'lane_detection_node'

# Topics subcribed/published to in this program
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'

SUBSCRIBER_TOPIC_NAME = '/scan'


class LaneDetection(Node):
    def __init__(self):

        #Added Code
        self.lastDetection = 0
        self.centerLaneMiddle = 0.5

        #Variables that can be Changed
        self.centerLaneInner = 0.2
        self.centerLaneOuter = 0.8
        self.errorRightTurn = 1.0
        self.errorLeftTurn = -1.0
        self.delayTime = 4500



        super().__init__(NODE_NAME)
        self.centroid_error_publisher = self.create_publisher(Float32, CENTROID_TOPIC_NAME, 10)
        self.centroid_error_publisher
        self.centroid_error = Float32()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.camera_subscriber
        self.bridge = CvBridge()
        self.max_num_lines_detected = 10
        self.image_width = 0
        self.image_height = 0
        self.start_height = 0
        self.bottom_height = 0
        self.left_width = 0
        self.right_width = 0
        self.error_threshold = 0.1
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Hue_low', 1),
                ('Hue_high', 1),
                ('Saturation_low', 1),
                ('Saturation_high', 1),
                ('Value_low', 1),
                ('Value_high', 1),
                ('gray_lower', 1),
                ('inverted_filter', 0),
                ('kernal_size',1),
                ('erosion_itterations',1),
                ('dilation_itterations',1),
                ('number_of_lines', 0),
                ('error_threshold', 0),
                ('Width_min', 1),
                ('Width_max', 1),
                ('crop_width_decimal',0.8),
                ('rows_to_watch_decimal',0.2),
                ('rows_offset_decimal',0.5),
                ('camera_centerline',0.5),
                ('debug_cv', 0)
            ])
        self.Hue_low = self.get_parameter('Hue_low').value
        self.Hue_high = self.get_parameter('Hue_high').value
        self.Saturation_low = self.get_parameter('Saturation_low').value
        self.Saturation_high = self.get_parameter('Saturation_high').value
        self.Value_low = self.get_parameter('Value_low').value
        self.Value_high = self.get_parameter('Value_high').value
        self.gray_lower = self.get_parameter('gray_lower').value
        self.inverted_filter = self.get_parameter('inverted_filter').value
        self.kernal_size = self.get_parameter('kernal_size').value
        self.erosion_itterations = self.get_parameter('erosion_itterations').value
        self.dilation_itterations = self.get_parameter('dilation_itterations').value
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.min_width = self.get_parameter('Width_min').value
        self.max_width = self.get_parameter('Width_max').value
        self.crop_width_decimal = self.get_parameter('crop_width_decimal').value
        self.rows_to_watch_decimal = self.get_parameter('rows_to_watch_decimal').value
        self.rows_offset_decimal = self.get_parameter('rows_offset_decimal').value
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.debug_cv = self.get_parameter('debug_cv').value
        
        self.camera_init = False
        
        self.get_logger().info(
            f'\nHue_low: {self.Hue_low}'
            f'\nHue_high: {self.Hue_high}'
            f'\nSaturation_low: {self.Saturation_low}'
            f'\nSaturation_high: {self.Saturation_high}'
            f'\nValue_low: {self.Value_low}'
            f'\nValue_high: {self.Value_high}'
            f'\ngray_lower: {self.gray_lower}'
            f'\ninverted_filter: {self.inverted_filter}'
            f'\nkernal_size: {self.kernal_size}'
            f'\nerosion_itterations: {self.erosion_itterations}'
            f'\ndilation_itterations: {self.dilation_itterations}'
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}'
            f'\ncrop_width_decimal: {self.crop_width_decimal}'
            f'\nrows_to_watch_decimal: {self.rows_to_watch_decimal}'
            f'\nrows_offset_decimal: {self.rows_offset_decimal}'
            f'\ncamera_centerline: {self.camera_centerline}'
            f'\ndebug_cv: {self.debug_cv}')

    def locate_centroid(self, data):
        # Image processing from rosparams
        #Added Code
        frame = self.bridge.imgmsg_to_cv2(data)
        boolOrange = orangeMatching(frame)

        if not self.camera_init:
            self.get_logger().info(f'\n Initializing Camera...')
            height, width, channels = frame.shape
        
            # Vertical crop/pan
            rows_to_watch = int(height * self.rows_to_watch_decimal)
            rows_offset = int(height * (1 - self.rows_offset_decimal))

            # Horizontal crop
            self.start_height = int(height - rows_offset)
            self.bottom_height = int(self.start_height + rows_to_watch)
            self.left_width = int((width / 2) * (1 - self.crop_width_decimal))
            self.right_width = int((width / 2) * (1 + self.crop_width_decimal))
            self.camera_init = True
            self.get_logger().info(f'\n Camera Initialized')

        self.image_width = int(self.right_width - self.left_width)
        self.image_height = self.bottom_height-self.start_height
        img = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([self.Hue_low, self.Saturation_low, self.Value_low])
        upper = np.array([self.Hue_high, self.Saturation_high, self.Value_high])
        mask = cv2.inRange(hsv, lower, upper)

        if self.inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=mask)

        # changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        
        # get rid of white noise from grass
        kernel = np.ones((self.kernal_size, self.kernal_size), np.uint8)
        blurred = cv2.blur(blackAndWhiteImage,(self.kernal_size, self.kernal_size))
        erosion = cv2.erode(blurred, kernel, iterations = self.erosion_itterations)
        dilation = cv2.dilate(erosion, kernel, iterations = self.dilation_itterations)

        (dummy, blackAndWhiteImage) = cv2.threshold(dilation, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Defining points of a line to be drawn for visualizing error
        cam_center_line_x = int(self.image_width * self.camera_centerline)
        start_point = (cam_center_line_x,0)
        end_point = (cam_center_line_x, int(self.bottom_height))
        
        start_point_thresh_pos_x = int(cam_center_line_x -  (self.error_threshold * self.image_width/2))
        start_point_thresh_neg_x = int(cam_center_line_x + (self.error_threshold * self.image_width/2))
        
        start_point_thresh_pos = (start_point_thresh_pos_x, 0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(self.bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x, 0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(self.bottom_height))

        # Setting up data arrays
        cx_list = []
        cy_list = []

        #Added Code
        currentDetection = int(time.time() * 1000)

        
        for contour in contours[:self.number_of_lines]:
            [x, y], [w, h], phi = cv2.minAreaRect(contour)
            rect = cv2.minAreaRect(contour)
            if self.min_width < w < self.max_width:
                try:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    img = cv2.drawContours(img,[box], 0, (0, 255, 0), 3)
                    m = cv2.moments(contour)
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        
        try:        
            
            if len(cx_list) > 1:
                error_list = []
                count = 0
                
                for cx_pos in cx_list:
                    error = float((cx_pos - cam_center_line_x) / cam_center_line_x)
                    error_list.append(error)
                
               
                avg_error = (sum(error_list) / float(len(error_list)))

               
                p_horizon_diff = abs(error_list[0] - error_list[-1])

               
                if abs(p_horizon_diff) <= self.error_threshold:
                    error_x = avg_error
                    pixel_error = int(cam_center_line_x * (1 + error_x))
                    mid_x, mid_y = pixel_error, int((self.image_height/2))
                    self.get_logger().info(f"Straight curve: [tracking error: {error_x}], [tracking angle: {phi}], [Orange: {boolOrange}]")
                    
                    if (boolOrange) and (currentDetection - self.lastDetection > self.delayTime):
                        if self.camera_centerline > self.centerLaneMiddle:
                            self.camera_centerline = self.centerLaneInner
                        else:
                            self.camera_centerline = self.centerLaneOuter
                        self.lastDetection = currentDetection

                
                else:
                   
                    for error in error_list:
                        if abs(error) < self.error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1
                    
                    
                    error_x = min(error_list, key=abs)

                   
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    self.get_logger().info(f"Curvy road: [tracking error: {error_x}], [tracking angle: {phi}], [Orange: {boolOrange}]")            
                   
                    if (boolOrange) and (currentDetection - self.lastDetection > self.delayTime):
                        if self.camera_centerline > self.centerLaneMiddle:
                            self.camera_centerline = self.centerLaneInner
                        else:
                            self.camera_centerline = self.centerLaneOuter
                        self.lastDetection = currentDetection

                
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)

               
                self.centroid_error.data = float(error_x)
                self.centroid_error_publisher.publish(self.centroid_error)

            # When only 1 road mark was found 
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                error_x = float((mid_x - cam_center_line_x) / cam_center_line_x)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
                
                self.centroid_error.data = error_x
                self.centroid_error_publisher.publish(self.centroid_error)
                self.get_logger().info(f"Only detected one line: [tracking error: {error_x}], [tracking angle: {phi}], [Orange: {boolOrange}]")
                #Added Code
                if (boolOrange) and (currentDetection - self.lastDetection > self.delayTime):
                    if self.camera_centerline > self.centerLaneMiddle:
                        self.camera_centerline = self.centerLaneInner
                    else:
                        self.camera_centerline = self.centerLaneOuter
                    self.lastDetection = currentDetection


            # When Nothing was found
            else:
                self.get_logger().info(f"Nothing detected, [Orange: {boolOrange}]")
                if(self.camera_centerline > self.centerLaneMiddle):
                    self.centroid_error.data = self.errorRightTurn
                    self.centroid_error_publisher.publish(self.centroid_error)
                else:
                    self.centroid_error.data = self.errorLeftTurn      
                    self.centroid_error_publisher.publish(self.centroid_error)        

            # clean slate
            error_list = [0] * self.number_of_lines
            cx_list = []
            cy_list = []
        except ValueError:
            pass

        # plotting results
        self.debug_cv = self.get_parameter('debug_cv').value # ability to update debug in real-time
        if self.debug_cv:
            cv2.imshow('img', img)
            cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()


def orangeMatching(inputImage):
    orange_lower = np.array([0, 50, 50])
    orange_upper = np.array([20, 255, 255])
    hsv_frame = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)
    total_pixels = mask.size
    orange_pixels = np.count_nonzero(mask)
    orange_percentage = (orange_pixels / total_pixels) * 100
    if orange_percentage >= 7.5:
        return 1
    else:
        return 0



def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = LaneDetection()
    try:
        rclpy.spin(centroid_publisher)
        centroid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        centroid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        centroid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
