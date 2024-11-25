"""
This project is designed to park a robot in a corner in a specific location.
It is assumed that the robot is facing a corner and that the laserscanner can
detect both walls.
This project has been tested using the Argo robot.

If Python 3.10 is used, then a Switch statement can be used for the self.state 
sections of the program to the replace the if statements. 

This project was created using ROS2 Humble

This project was created using trigonometry. Another way of completing this
project is by using the ROS2 path planning service.
"""

# ROS 2 specific includes
# Important: these includes should be reflected in the package.xml
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

class Drive(Node):

    # Constructor, specifying node name for the parent class
    def __init__(self):
        super().__init__('drive') # TODO: Update node name to reflect purpose of node

        # Publisher for outputting text to cmd to show data at each stage of the program
        self.publisher_ = self.create_publisher(String, '/hello/world', 10)
        # Publisher for the robot to drive
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)
        # Subscriprion to laserscanner data
        self.subscription = self.create_subscription(
            LaserScan,
            '/front_laser/scan',
            self.laser_callback,
            10)
        
        timer_period = 0.5  # seconds

        # Create a timer that will trigger calls to the method timer_callback
        # at the frequency specified above
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.state = 1
        self.goLeft = False
        self.linVelocity = 0.0
        self.angVelocity = 0.0
        self.directionForwards = 0

        # Create a counter for the purpose of the messages being sent
        self.i = 0

    def timer_callback(self):
        # Create an instance of the appropriate message and driving types
        msg = String()
        drive = Twist()

        # Set the values for necessary fields in the message type
        msg.data = 'Hello, world!'

        # publish the message to the topic
        self.publisher_.publish(msg)
        

        # Log output for published message
        self.get_logger().info('Publishing: "%s"' % msg.data)

        #Driving velocity is based on the distance to the wall. The robot moves slower the closer it is to the wall.
        if self.goLeft == True and self.state == 3:
            drive.linear.x = self.linVelocity/20
            drive.angular.z = 0 - self.angVelocity

        elif self.goLeft == False and self.state == 3:
            drive.angular.z = self.angVelocity
            drive.linear.x = self.linVelocity/20

        elif self.state > 3:
            drive.linear.x = self.linVelocity/10
            drive.angular.z = self.angVelocity


        
        self.publisher.publish(drive)


    def laser_callback(self, msg):
        # Check if there is a wall left and the wall's gradient
        distanceToWalls = 1.0
        if self.state == 1:
            angledDistance = msg.ranges[29]
            distanceAt0degrees = msg.ranges[0]
            
            height = angledDistance*math.sin(math.radians(30))
            length = angledDistance*math.cos(math.radians(30))

            triangle = 1
            if length != distanceAt0degrees:
                newLength = distanceAt0degrees - length
                triangle = 2
        
            if triangle == 1:
                gradientOne = height/length
                self.get_logger().info('GradientOne: "%s"' % gradientOne)
            elif triangle == 2:
                gradientOne = height/newLength
                self.get_logger().info('GradientOne: "%s"' % gradientOne)
            
            else:
                self.get_logger().info('Not Detected triangle A')

            angledDistance = msg.ranges[54]
            heightOfTriangleThree = angledDistance*math.sin(math.radians(55))
            lengthOfTriangleThree = angledDistance*math.cos(math.radians(55))
            triangle = 3

            if lengthOfTriangleThree != distanceAt0degrees:
                newLength = distanceAt0degrees - lengthOfTriangleThree
                triangle = 4
        
            if triangle == 3:
                gradientTwo = heightOfTriangleThree/lengthOfTriangleThree
                self.get_logger().info('GradientTwo: "%s"' % gradientTwo)
            elif triangle == 4:
                gradientTwo = heightOfTriangleThree/newLength
                self.get_logger().info('GradientTwo: "%s"' % gradientTwo)
            
            else:
                self.get_logger().info('Not Detected triangle C')

    
        
            gradientOneMin = gradientOne - 0.05
            gradientOneMax = gradientOne + 0.05

            if gradientTwo > gradientOneMin and gradientTwo < gradientOneMax:
                self.get_logger().info('Wall left')
                wallLeft = True
            else:
                self.get_logger().info('Not a wall left of robot')
                wallLeft = False
        
        
            heightOfTriangleThree = angledDistance*math.sin(math.radians(55))
            lengthOfTriangleThree = angledDistance*math.cos(math.radians(55))
            triangle = 3

            if lengthOfTriangleThree != distanceAt0degrees:
                newLength = distanceAt0degrees - lengthOfTriangleThree
                triangle = 4
        
            if triangle == 3:
                gradientTwo = heightOfTriangleThree/lengthOfTriangleThree
                self.get_logger().info('GradientTwo: "%s"' % gradientTwo)
            elif triangle == 4:
                gradientTwo = heightOfTriangleThree/newLength
                self.get_logger().info('GradientTwo: "%s"' % gradientTwo)
            
            else:
                self.get_logger().info('Not Detected triangle C')

    
        
            gradientOneMin = gradientOne - 0.05
            gradientOneMax = gradientOne + 0.05

            if gradientTwo > gradientOneMin and gradientTwo < gradientOneMax:
                self.get_logger().info('Wall left')
                wallLeft = True
            else:
                self.get_logger().info('Not a wall left of robot')
                wallLeft = False

            #Check if there is a Wall right to the robot and the wall's gradient

            angledDistance = msg.ranges[119]
            distanceAt180degrees = msg.ranges[179]
            

            height = angledDistance*math.sin(math.radians(60))
            length = angledDistance*math.cos(math.radians(60))
            triangle = 5

            if length != distanceAt180degrees:
                newLength = distanceAt180degrees - length
                triangle = 6
        
            if triangle == 5:
                gradientThree = height/length
                self.get_logger().info('GradientThree: "%s"' % gradientThree)
            elif triangle == 6:
                gradientThree = height/newLength
                self.get_logger().info('GradientThree: "%s"' % gradientThree)
            
            else:
                self.get_logger().info('Not Detected triangle C')

            angledDistance = msg.ranges[149]
            height = angledDistance*math.sin(math.radians(30))
            length = angledDistance*math.cos(math.radians(30))
            triangle = 7
            gradientFour = 0

            if length != distanceAt180degrees:
                newLength = distanceAt180degrees - length
                triangle = 8
        
            if triangle == 7:
                gradientFour = height/length
                self.get_logger().info('GradientFour: "%s"' % gradientFour)
            elif triangle == 8:
                gradientFour = height/newLength
                self.get_logger().info('GradientFour: "%s"' % gradientFour)
            
            else:
                self.get_logger().info('Not Detected triangle C')

        


            gradientThreeMin = gradientThree - 0.05
            gradientThreeMax = gradientThree + 0.05

            if gradientFour> gradientThreeMin and gradientFour < gradientThreeMax:
                self.get_logger().info('Wall right')
                wallRight = True
            else:
                self.get_logger().info('Not a wall right of robot')
                wallRight = False
            #Is it a corner?
            if wallRight == True and wallLeft == True:
                cornergradient = 1/gradientFour
                gradientOneMin = gradientOneMin - 0.1
                gradientOneMax = gradientOneMax + 0.1
                self.get_logger().info('cornergradient: "%s"' % cornergradient)
                if cornergradient > gradientOneMin and cornergradient < gradientOneMax:
                    self.get_logger().info('Corner')
                    self.state = 2
                else:
                    self.get_logger().info('Not a corner')
            else:
                self.get_logger().info('Not a corner')

        #Should the robot drive left or right
        #This is determined using the distances of either side of the robot.
        elif self.state == 2:
            leftDistance = msg.ranges[0]
            rightDistance = msg.ranges[179]
            if leftDistance < rightDistance:
                self.goLeft = True
            else:
                self.goLeft = False
            self.state = 3

        #Robot turns towards closest wall
        elif self.state == 3:
            if self.goLeft == True:
                self.get_logger().info('Left')
                self.linVelocity = msg.ranges[89]
                self.angVelocity = -0.5
                if msg.ranges[105] < msg.ranges[126]:
                    self.linVelocity = 0.0
                    self.angVelocity = 0.5
                    self.state = 4
                
            else:
                self.get_logger().info('Right')
                self.get_logger().info('linVel: "%s"' % self.linVelocity)
                self.linVelocity = msg.ranges[89]
                self.angVelocity = -0.5
                if msg.ranges[75] < msg.ranges[54]:
                    self.linVelocity = 0.0
                    self.angVelocity = 0.5
                    self.state = 4
        
        #Align with wall
        elif self.state == 4:
            self.get_logger().info('linVel: "%s"' % self.linVelocity)
            self.get_logger().info('angVel: "%s"' % self.angVelocity)
            currentDistance = msg.ranges[89]
            checkLeft = msg.ranges[80]
            checkRight = msg.ranges[100]
            distanceForSide = distanceToWalls + 0.78 #+1
            currentDistanceMax = currentDistance + 0.1
            currentDistanceMin = currentDistance - 0.1
            if distanceForSide < currentDistanceMax:
                self.linVelocity = currentDistance
                self.directionForwards = 1
            elif distanceForSide > currentDistanceMin:
                self.get_logger().info('distance: "%s"' % currentDistanceMin)
                self.get_logger().info('wall: "%s"' % distanceForSide)
                self.linVelocity = 0 - currentDistance
                self.directionForwards = -1
            else:
                self.linVelocity = 0.0
                self.directionForwards = 0
            
            checkOne =checkLeft - checkRight
            checkTwo = checkRight - checkLeft
            
            #velocity determined based on the following conditions.
            if distanceForSide > currentDistanceMin and distanceForSide < currentDistanceMax:
                CheckLocation = True
            else:
                CheckLocation = False


            if CheckLocation == True and checkOne < 0.05 and checkOne > 0.0:
                self.state = 5
                self.angVelocity = 0.1
            
            elif CheckLocation == True and checkTwo< 0.05 and checkTwo > 0.0:
                self.state = 5
                self.angVelocity = 0.1

            elif checkRight > checkLeft and self.directionForwards == 1:
                self.angVelocity = checkOne
            
            elif checkLeft > checkRight and self.directionForwards == -1:
                self.angVelocity = checkOne
            
            elif checkRight > checkLeft and self.directionForwards == -1:
                self.angVelocity = checkTwo 
            elif checkLeft > checkRight and self.directionForwards == 1:
                self.angVelocity = 0 - checkTwo
            
        #Rotate to other wall.
        elif self.state == 5:
            if self.goLeft == False:
                distLeft = msg.ranges[169]
                if math.isinf(distLeft):
                    self.angVelocity = 0.1
                    self.linVelocity = 0.0
                    self.state = 6
                else:
                    self.angVelocity = 0.5
                    self.linVelocity = 3
            
                self.get_logger().info('linVel: "%s"' % self.linVelocity)
                self.get_logger().info('angVel: "%s"' % self.angVelocity)
            else:
                distRight = msg.ranges[11]
                if math.isinf(distRight):
                    self.angVelocity = 0.1
                    self.linVelocity = 0.0
                    self.state = 6
                else:
                    self.angVelocity = 0.5
                    self.linVelocity = 3
            
                self.get_logger().info('linVel: "%s"' % self.linVelocity)
                self.get_logger().info('angVel: "%s"' % self.angVelocity)

        #Align with walls
        elif self.state == 6:
            checkLeft = msg.ranges[79]
            checkRight = msg.ranges[99]
            distance = msg.ranges[89]
            
            self.get_logger().info('distance: "%s"' % distance)
            currentDistanceMax = distance + 0.05
            currentDistanceMin = distance - 0.05
            if distanceToWalls < currentDistanceMax:
                self.linVelocity = distance
                self.directionForwards = 1
            elif distanceToWalls > currentDistanceMin:
                self.get_logger().info('distance: "%s"' % currentDistanceMin)
                self.linVelocity = 0 - distance
                self.directionForwards = -1
            else:
                self.linVelocity = 0.0
                self.directionForwards = 0
            
            checkOne =checkLeft - checkRight
            checkTwo = checkRight - checkLeft
            
        
            if distanceToWalls > currentDistanceMin and distanceToWalls < currentDistanceMax:
                CheckLocation = True
            else:
                CheckLocation = False


            if CheckLocation == True and checkOne < 0.05 and checkOne > 0.0:
                self.angVelocity = 0.1
            
            elif CheckLocation == True and checkTwo< 0.05 and checkTwo > 0.0:
                self.angVelocity = 0.1

            elif checkRight > checkLeft and self.directionForwards == 1:
                self.angVelocity = checkOne
            
            elif checkLeft > checkRight and self.directionForwards == -1:
                self.angVelocity = -checkOne
            
            elif checkRight > checkLeft and self.directionForwards == -1:
                self.angVelocity = checkTwo 
            elif checkLeft > checkRight and self.directionForwards == 1:
                self.angVelocity = 0 - checkTwo

        else:
            self.get_logger().info('state not initiated')
                
            

# Main method to be defined as entry point in setup.py
def main(args=None):

    # Initialise ROS 2 for this node
    rclpy.init(args=args)

    # Create an instance of the node subclass defined above
    drive = Drive()

    # Start the ROS 2 spinner with a reference to the node subclass
    rclpy.spin(drive)

    # Destroy the node and shutdown ROS 2
    drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
