# ROS 2 specific includes
# Important: these includes should be reflected in the package.xml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String 

''' 
Template giving outline for creating a program
that contains both a publisher and subscriber.
'''

    # Constructor, specifying node name for the parent class
class SensorTest(Node):
    def __init__(self):
        super().__init__("sensor_test")
        
        self.sensorTestSubscription = self.create_subscription(
            LaserScan,
            '/front_laser/scan',
            self.laser_callback,
            10)
        
        #timer_period = 0.5 #seconds
        # Define laserscan message parameters (similar to C++ example)
        # ... (
        
        timer_period = 0.5  # seconds

        # Create a timer that will trigger calls to the method timer_callback
        # at the frequency specified above
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a counter for the purpose of the messages being sent
        #self.i = 0

    def timer_callback(self):
        # TODO: add behaviour here to be repeated at regular intervals
        
        # Create an instance of the appropriate message type
        msg = String()

        # Set the values for necessary fields in the message type
        msg.data = 'Hello, world!'

        # publish the message to the topic
        self.publisher_.publish(msg)

        # Log output for published message
        self.get_logger().info('Publishing: "%s"' % msg.data)
    # Display the laserscanner data from 3 directions.
    def laser_callback(self, msg):
        # Logger used to print the details of the message received (printed to console)
        self.get_logger().info('I heard from right: "%s"' % msg.ranges[0])
        self.get_logger().info('I heard from forward: "%s"' % msg.ranges[90])
        self.get_logger().info('I heard from the left: "%s"' % msg.ranges[179])
        
        
        

# Main method to be defined as entry point in setup.py
def main(args=None):

    # Initialise ROS 2 for this node
    rclpy.init(args=args)

    # Create an instance of the node subclass defined above
    sensorTest = SensorTest()

    # Start the ROS 2 spinner with a reference to the node subclass
    rclpy.spin(sensorTest)

    # Destroy the node and shutdown ROS 2
    sensorTest.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
