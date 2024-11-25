# ROS 2 specific includes
# Important: these includes should be reflected in the package.xml
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# TODO: Add or replace with message type needed
from std_msgs.msg import String 



''' 
This program determines if there is a wall on either side of the robot
and if the walls join together to make a corner. The robot then checks 
which wall the robot is closest too.
'''
class Navigation(Node):

    # Constructor, specifying node name for the parent class
    def __init__(self):
        super().__init__('navigation') # TODO: Update node name to reflect purpose of node

        # Publisher for outputting text to cmd to show data at each stage of the program
        # Publisher for the robot to drive
        self.publisher_ = self.create_publisher(Twist, '/servoing_cmd_vel',10)
        self.publisher= self.create_publisher(String, '/hello/world', 10)
        # Subscriprion to laserscanner data
        self.subscription = self.create_subscription(
            LaserScan,
            '/front_laser/scan',
            self.laser_callback,
            10)
        
        # TODO: Update frequency to trigger recurring behaviour
        timer_period = 0.5  # seconds
        
        self.state = 1


        # Create a timer that will trigger calls to the method timer_callback
        # at the frequency specified above
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a counter for the purpose of the messages being sent
        self.i = 0

    def timer_callback(self):
        
        # Create an instance of the appropriate message type
        msg = String()
        

        # Set the values for necessary fields in the message type
        msg.data = 'Hello, world!'

        # publish the message to the topic
        self.publisher_.publish(msg)

        # Log output for published message
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def laser_callback(self, msg):

        if self.state == 1:
            distanceAt30degrees = msg.ranges[29]
            distanceAt0degrees = msg.ranges[0]
            distanceAtFiftyFivedegrees = msg.ranges[54]

            height = distanceAt30degrees*math.sin(math.radians(30))
            length = distanceAt30degrees*math.cos(math.radians(30))

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

            heightOfTriangleThree = distanceAtFiftyFivedegrees*math.sin(math.radians(55))
            lengthOfTriangleThree = distanceAtFiftyFivedegrees*math.cos(math.radians(55))
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




            #Is there a wall to the rightod the robot

            distanceAt120degrees = msg.ranges[119]
            distanceAt180degrees = msg.ranges[179]
            distanceAt150degrees = msg.ranges[149]

            height = distanceAt120degrees*math.sin(math.radians(60))
            length = distanceAt120degrees*math.cos(math.radians(60))
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


        
        
            height = distanceAt150degrees*math.sin(math.radians(30))
            length = distanceAt150degrees*math.cos(math.radians(30))
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
        
        
            if wallRight == True and wallLeft == True:
                cornergradient = 1/gradientFour
                gradientOneMin = gradientOneMin - 0.1
                gradientOneMax = gradientOneMax + 0.1
                self.get_logger().info('cornergradient: "%s"' % cornergradient)
                if cornergradient > gradientOneMin and cornergradient < gradientOneMax:
                    self.get_logger().info('Corner')
                    state = 2
                else:
                    self.get_logger().info('Not a corner')
            else:
                self.get_logger().info('Not a corner')
        
        #Should the robot drive left or right
        #This is determined using the distances of either side of the robot.
        elif self.state == 2:
            leftDistance = msg.ranges[0]
            rightDistance = msg.ranges[179]
            self.get_logger().info('distance to left: "%s"' % leftDistance)
            self.get_logger().info('distance to right: "%s"' % rightDistance)
        
        else:
            self.get_logger().info('state not initiated')



        


        

    
        

        



# Main method to be defined as entry point in setup.py
def main(args=None):

    # Initialise ROS 2 for this node
    rclpy.init(args=args)

    # Create an instance of the node subclass defined above
    navigation = Navigation()

    # Start the ROS 2 spinner with a reference to the node subclass
    rclpy.spin(Navigation)

    # Destroy the node and shutdown ROS 2
    Navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
