#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import os
import math
import time
import numpy as np

class TurtleTracker(Node):
    def __init__(self, points, resoulution, time_limit):
        """
            Tracks the position of the turtle and calculates the percentage of the area it cleansed in the defined time between the given points

            Parameters:
                points (list): List of points that define the area
                resoulution (float): Resolution of the map
                time_limit (int): Time limit in seconds
            Returns:
                None
        """

        # Initialize the node
        super().__init__('turtle_tracker')
        self.given_points = points
        self.time_limit = time_limit
        self.resolution = resoulution

        # Create the Twist publisher and Pose subscriber
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.turtle_pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.turtlepose_callback, 10)
        self.turtle_pose_sub_ # prevent unused variable warning
            
        # Initialize the start time
        self.start_time = time.time()

        # Initialize the map, total area, cleaned area and the ratio
        self.map = self.create_map(self.given_points)
        self.total_area = self.calculate_total_area(self.given_points, self.resolution)
        self.cleaned_area = 0
        self.ratio = self.cleaned_area / self.total_area 

        # Initialize the record flag
        self.record = True

    def turtlepose_callback(self, msg):
        """
            Callback function for the Pose subscriber.

            Parameters:
                msg (Pose): Current turtle pose
            Returns:
                None
        """

        # Check if the time limit is reached
        elapsed_time = self.calculate_elapsed_time(self.start_time)
        if elapsed_time < self.time_limit:
            #self.get_logger().info('Cleaning in progress')
            #self.get_logger().info('Elapsed time: {:.2f} seconds'.format(elapsed_time))
            #self.get_logger().info('Percentage of cleaned area: %f' % self.ratio)

            # Check if the turtle is inside the given area
            if self.is_inside_target_area(self.given_points, msg.x, msg.y):

                # Check if the current position is already cleaned
                floor_x = math.floor(msg.x * 10)
                ceil_y = math.ceil(msg.y * 10)
                x = abs(40 - floor_x)
                y = abs(70 - ceil_y)
                
                # If the current position is not cleaned, clean it
                if self.map[x][y] == 0:
                    self.map[x][y] = 1
                    self.cleaned_area += 1
                    self.ratio = self.cleaned_area / self.total_area
        else:
            # If the time limit is reached, stop the turtle and print the result
            #self.get_logger().info('Time is up. Final percentage of cleaned area is %f' % self.ratio)
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            if self.record:
                self.get_logger().info('Recording the result')
                self.record = False
                with open(f'/home/{os.getlogin()}/ros2_ws/src/assignment1/assignment1.txt', 'w') as file:
                    file.write(str(self.ratio))

        #self.get_logger().info('----------------------------------------------------------')

    def calculate_elapsed_time(self, start_time):
        """
            Calculate the elapsed time since the start of the program.

            Parameters:
                None
            Returns:
                Elapsed time since the start of the program
        """
        return (time.time() - start_time)

    def create_map(self, points):
        """
            Create a map of the given area.

            Parameters:
                points (list): List of points that define the area
            Returns:
                Map of the given area
        """
        x_min = min(p[0] for p in points)
        x_max = max(p[0] for p in points)
        y_min = min(p[1] for p in points)
        y_max = max(p[1] for p in points)

        rows = int((x_max - x_min) / self.resolution)
        columns = int((y_max - y_min) / self.resolution)

        return np.zeros((rows + 1, columns + 1))

    def calculate_total_area(self, points, resolution=0.1):
        """
            Calculate the total area of the given points.

            Parameters:
                points (list): List of points that define the area
                resolution (float): Resolution of the map
            Returns:
                Total area of the given points
        """
        x_min = min(p[0] for p in points)
        x_max = max(p[0] for p in points)
        y_min = min(p[1] for p in points)
        y_max = max(p[1] for p in points)

        rows = int((x_max - x_min) / resolution)
        columns = int((y_max - y_min) / resolution)

        return rows * columns

    def is_inside_target_area(self, points, x, y, margin=0.1):
        """
            Check if the current turtles position is inside the given area.

            Parameters:
                x (float): Current x position
                y (float): Current y position
                margin (float): Margin to expand the target area
            Returns:
                True if the current turtles position is inside the target area, False otherwise
        """
        min_x = min(p[0] for p in points) - margin
        max_x = max(p[0] for p in points) + margin
        min_y = min(p[1] for p in points) - margin
        max_y = max(p[1] for p in points) + margin

        return (x >= min_x and x <= max_x and y >= min_y and y <= max_y)
    
def main(args=None):
    rclpy.init(args=args)
    points =[[7.0, 7.0], [7.0, 4.0], [4.0, 4.0], [4.0, 7.0]]
    resolution = 0.1
    time_limit = 120
    turtle_tracker = TurtleTracker(points, resolution, time_limit)
    rclpy.spin(turtle_tracker)
    rclpy.shutdown()

if __name__ == '__main__':
    main()