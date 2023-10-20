#!/usr/bin/env python3

########## FILL HERE ##########
# NAME & SURNAME: İrem Çağın Yurttürk
# STUDENT ID: 150220765
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
        self.firstMove = True

        # Start the cleaning path
        self.cleaningPath(self.area)

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
        #self.get_logger().info('x: %f' % msg.x)
        #self.get_logger().info('y: %f' % msg.y)
	
        pass
        ########################################

    
    # Calculate distance between two points
    def calculateDistance(self,x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance


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
        startPos = self.currentPose

        # While current position minus starting position (to get exact changed position) is less than distance, turtle moves linearly. 
        # Movement is made with publishing the twist msg
        while self.calculateDistance(self.currentPose.x,self.currentPose.y,startPos.x,startPos.y) < distance:
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

        # Setting the initial position
        startPosTheta = math.degrees(self.currentPose.theta)

        # Since turtles theta never becomes 180 degrees, this condition prevents infinite loop
        if(angle == 180):
            angle = angle - 1.0

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

        # -------------------------- ROTATION --------------------------
        # Receive the Pose information
        rclpy.spin_once(self)

        # Calculate the rotation angle where turtle should head
        desiredTheta = math.atan2(abs(point[1]) - self.currentPose.y, abs(point[0]) - self.currentPose.x)
        currentTheta = self.currentPose.theta
        rotationTheta = desiredTheta - currentTheta

        # Normalization of rotation angle between -pi and pi
        rotationTheta = math.atan2(math.sin(rotationTheta), math.cos(rotationTheta))

        if rotationTheta < 0:
            # Clockwise rotation
            self.rotate(-math.degrees(rotationTheta), angular_speed, True)
        else:
            # Counterclockwise rotation
            self.rotate(math.degrees(rotationTheta), angular_speed, False) 
        

        # -------------------------- MOVEMENT --------------------------
        # Receive the Pose information
        rclpy.spin_once(self)

        # Calculate distance between target and current point
        distance = self.calculateDistance(point[0],point[1],self.currentPose.x,self.currentPose.y)
        self.move(distance,linear_speed,True)

        pass
        ########################################
    
    ########## YOU CAN ADD YOUR FUNCTIONS HERE ##########
   

    def cleaningPath(self, points):
        linearSpeed = 5.0
        angularSpeed = 5.0

        # Determine the corners
        x_min = min(p[0] for p in points)
        x_max = max(p[0] for p in points)
        y_min = min(p[1] for p in points)
        y_max = max(p[1] for p in points)

        slip = 0.05
        tolerance = 0.5
        

        self.firstPattern(x_max,x_min,y_max,y_min,slip,tolerance,linearSpeed,angularSpeed)
        
        self.secondPattern(x_max,x_min,y_max,y_min,slip,tolerance,linearSpeed,angularSpeed)
        
        pass


    def firstPattern(self,x_max,x_min,y_max,y_min,slip,tolerance,linearSpeed, angularSpeed):
        rclpy.spin_once(self)
        currentX = x_max

        # Pattern : down - left- up - left
        while(self.firstMove | ((currentX > x_min) & 
              (self.currentPose.x <= x_max + tolerance)  & 
              (self.currentPose.x >= x_min - tolerance)  & 
              (self.currentPose.y <= y_max + tolerance)  & 
              (self.currentPose.y >= y_min - tolerance))):
            # Go to first point
            if(self.firstMove): 
                self.go_to_a_goal([x_max,y_max], linearSpeed, angularSpeed)
                self.firstMove = False
            # Down move
            self.go_to_a_goal([currentX,y_min], linearSpeed, angularSpeed)
            # Left move
            if(currentX - slip > x_min):
                currentX = currentX - slip
                self.go_to_a_goal([currentX,y_min], linearSpeed, angularSpeed)
                # Up move
                self.go_to_a_goal([currentX,y_max], linearSpeed, angularSpeed)
                # Left move
                if(currentX - slip > x_min):
                    currentX = currentX - slip
                    self.go_to_a_goal([currentX,y_max], linearSpeed, angularSpeed)

            else: break

            # Receive the Pose information
            rclpy.spin_once(self)

        self.firstMove = True
        # Stop the turtle after the loop
        self.twistMsg.linear.x = 0.0
        self.twistMsg.angular.z = 0.0
        self.twist_publisher_.publish(self.twistMsg)

        
        pass

    def secondPattern(self,x_max,x_min,y_max,y_min,slip,tolerance,linearSpeed, angularSpeed):
        rclpy.spin_once(self)
        currentY = y_max

        # Pattern : right - down - left - down
        while(self.firstMove | ((currentY > y_min) & 
              (self.currentPose.x <= x_max + tolerance)  & 
              (self.currentPose.x >= x_min - tolerance)  & 
              (self.currentPose.y <= y_max + tolerance)  & 
              (self.currentPose.y >= y_min - tolerance))):
            # Go to first point
            if(self.firstMove): 
                self.go_to_a_goal([x_min,y_max], linearSpeed, angularSpeed)
                self.firstMove = False
            # Right move
            self.go_to_a_goal([x_max,currentY], linearSpeed, angularSpeed)
            # Down move
            if(currentY - slip > y_min):
                currentY = currentY - slip
                self.go_to_a_goal([x_max,currentY], linearSpeed, angularSpeed)
                # Left move
                self.go_to_a_goal([x_min,currentY], linearSpeed, angularSpeed)
                # Down move
                if(currentY - slip > y_min):
                    currentY = currentY - slip
                    self.go_to_a_goal([x_min,currentY], linearSpeed, angularSpeed)
                 

            else: break

            # Receive the Pose information
            rclpy.spin_once(self)

        # Stop the turtle after the loop
        self.twistMsg.linear.x = 0.0
        self.twistMsg.angular.z = 0.0
        self.twist_publisher_.publish(self.twistMsg)
       
        pass


    pass
    #####################################################


########## DO NOT EDIT ANYTHING BELOW THIS LINE ##########
def main(args=None):
    rclpy.init(args=args)
    points =[[7.0, 7.0], [7.0, 4.0], [4.0, 4.0], [4.0, 7.0]]
    turtle_cleaner = TurtleCleaner(points)

    #turtle_cleaner.go_to_a_goal([5.0,5.0], 0.8, 1.0)
 
    rclpy.spin(turtle_cleaner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
