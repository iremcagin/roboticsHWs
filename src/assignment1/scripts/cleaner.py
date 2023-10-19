#!/usr/bin/env python3

########## FILL HERE ##########
# NAME & SURNAME:
# STUDENT ID:
###############################

########## DO NOT EDIT THESE LIBRARIES ##########
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
#################################################

########## ADD YOUR LIBRARIES HERE ##########

###########################################

class TurtleCleaner(Node):
    def __init__(self, points):
        """
            Cleans the given area with the turtle

            Parameters:
                points (list): List of points that define the area
            Returns:
                None
        """

        ########## DO NOT EDIT ANYTHING HERE ##########
        # Initialize the node
        super().__init__('turtle_cleaner')
        self.area = points

        # Create the twist_ publisher and Pose subscriber
        self.twist_publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.turtle_pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        #self.turtle_pose_sub_
        ###############################################

        ########## ADD YOUR CODE HERE ##########
        self.twistMsg = Twist()
        self.currentPose = Pose()

        pass
        ########################################

    def pose_callback(self, msg):
        """
            Callback function for the Pose subscriber.

            Parameters:
                msg (Pose): Current turtle pose
            Returns:
                None
        """
        ########## ADD YOUR CODE HERE ##########

        # This function gets called when a new Pose message is received
        self.currentPose = msg

        pass
        ########################################

    def move(self, distance, speed, isForward):
        """
            Moves the turtle with the given speed for the given distance

            Parameters: 
                distance (float): Distance to move
                speed (float): Speed to move
                isForward (bool): Direction of the movement (True: Forward, False: Backward)

            Returns:
                None
        """
        ########## ADD YOUR CODE HERE ##########

        # Decide the direction
        if isForward:      
            self.twistMsg.linear.x = abs(speed)
        else:
            self.twistMsg.linear.x = -abs(speed)

        # Receive the Pose information
        rclpy.spin_once(self)

        # Setting the initial position
        startPosX = self.currentPose.x

        # While current position minus starting position (to get exact changed position) is less than distance, turtle moves linearly. 
        # Movement is made with publishing the twist msg
        while abs(self.currentPose.x - startPosX) < distance:
            self.twist_publisher_.publish(self.twistMsg) 
            rclpy.spin_once(self)    # Execute a single iteration of the event loop to get new callback

        # Stop the turtle after the loop
        self.twistMsg.linear.x = 0.0
        self.twist_publisher_.publish(self.twistMsg)

        pass
        ########################################

    def rotate(self, angle, speed, isClockwise):
        """
            Rotates the turtle with the given speed for the given angle

            Parameters:
                angle (float): Angle to rotate in degrees
                speed (float): Speed to rotate
                isClockwise (bool): Direction of the rotation (True: Clockwise, False: Counter Clockwise)

        """
        ########## ADD YOUR CODE HERE ##########

        # Decide the direction
        if isClockwise:      
            self.twistMsg.angular.z = -abs(speed)
        else:
            self.twistMsg.angular.z = abs(speed)

        # Receive the Pose information
        rclpy.spin_once(self)

        # Setting the initial position in degrees
        startPosTheta = math.degrees(self.currentPose.theta)

        # While current orientation minus starting orientation (to get exact changed degree) is less than angle, turtle rotates according to choosen clock direction. 
        # Rotation is made with publishing the twist msg
        while abs(math.degrees(self.currentPose.theta) - startPosTheta) < angle:
            self.twist_publisher_.publish(self.twistMsg) 
            rclpy.spin_once(self)    # Execute a single iteration of the event loop to get new callback

        # Stop the turtle after the loop
        self.twistMsg.angular.z = 0.0
        self.twist_publisher_.publish(self.twistMsg)


        pass
        ########################################

    def go_to_a_goal(self, point, linear_speed, angular_speed):
        """
            Moves the turtle to the given point with the given speeds

            Parameters:
                point (list): Point to go
                linear_speed (float): Speed to move
                angular_speed (float): Speed to rotate in degrees

            Returns:
                None
        """
        ########## ADD YOUR CODE HERE ##########
        
        pass
        ########################################
    
    ########## YOU CAN ADD YOUR FUNCTIONS HERE ##########
    pass
    #####################################################


########## DO NOT EDIT ANYTHING BELOW THIS LINE ##########
def main(args=None):
    rclpy.init(args=args)
    points =[[7.0, 7.0], [7.0, 4.0], [4.0, 4.0], [4.0, 7.0]]
    turtle_cleaner = TurtleCleaner(points)

    turtle_cleaner.move(1,1.0,True)
    turtle_cleaner.rotate(45,0.8,False)
    
    rclpy.spin(turtle_cleaner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()