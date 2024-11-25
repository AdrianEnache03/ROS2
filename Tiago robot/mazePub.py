# ROS 2 specific includes
# Important: these includes should be reflected in the package.xml
import rclpy
import math
from rclpy.node import Node

# TODO: Add or replace with message type needed
from std_msgs.msg import String 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

''' 
This project is created for a obot to navigate around a maze.
'''

class MazePub(Node):
    
    # Constructor, specifying node name for the parent class
    def __init__(self):
        super().__init__('maze_pub')
        self.leftSide = 0
        self.rightSide = 0
        self.forwards = 0

        # Publisher for the robot to drive
        self.publisher_ = self.create_publisher(Twist, '/servoing_cmd_vel',10)
        # Subscriprion to laserscanner data
        self.scription = self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.laser_callback,
            10)
        

        timer_period = 0.5  # seconds
        self.array = [0,0,0]

        # Create a timer that will trigger calls to the method timer_callback
        # at the frequency specified above
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a counter for the purpose of the messages being sent
        self.i = 0
        #
    def timer_callback(self):
        # Create an instance for the robot to drive
        
        drive = Twist()
        if self.array[0] == 2:
              drive.angular.z = 0.098
        elif self.array[0] == 1:
              drive.angular.z = -0.098
        if self.array[1] == 2:
                drive.angular.z = 0.79
        elif self.array[1] == 1:
                drive.angular.z = -0.79
        linVelocity = self.array[2]/25.0*1.5
        drive.linear.x = linVelocity
        
        
        # publish the message to the topic
        self.publisher_.publish(drive)
        

    #Determines which way the robot should turn and at what speed.
    #Data is added to an array which timer_callback retrieves.
    def laser_callback(self, msg):       
        left = 0
        front = 222
        right = 443
        countRight = 0
        countForward = 0
        countLeft = 0
        rightStartCheck = 333
        rightEndCheck = 332
        self.append = []
        for i in range(front):
            if (not(math.isinf(msg.ranges[i]))):
                self.leftSide += msg.ranges[i]
                countLeft += 1
                #Drive forward
                #msgTwo.angular.z = 0.0
                #Identify which section the obstacle is in and then go in the opposite 
                #direction by turning left or right.
        
        self.leftSide /= countLeft
        for i in range(front,right):
            if (not(math.isinf(msg.ranges[i]))):
                self.forwards += msg.ranges[i]
                countForward += 1
        self.forwards /= countForward
        self.array[2] = self.forwards
        for i in range(right,665):
            if(not(math.isinf(msg.ranges[i]))):
                self.rightSide += msg.ranges[i]
                countRight += 1
        self.rightSide /= countRight
        
        
        if(self.leftSide > self.rightSide):
            self.array[0] = 1
            
        elif(self.rightSide > self.leftSide):
            self.array[0] = 2   
            
        if(self.leftSide>self.forwards):
            self.array[1] = 1
            
            
        elif(self.rightSide>self.forwards):
            
            self.array.append(2)
            self.array[1] = 2
        
        
        
        
    



# Main method to be defined as entry point in setup.py
def main(args=None):

    # Initialise ROS 2 for this node
    rclpy.init(args=args)

    # Create an instance of the node subclass defined above
    mazePub = MazePub()
    
    # Start the ROS 2 spinner with a reference to the node subclass
    rclpy.spin(mazePub)

    # Destroy the node and shutdown ROS 2
    mazePub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
