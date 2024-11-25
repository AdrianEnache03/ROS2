# ROS 2 specific includes
# Important: these includes should be reflected in the package.xml
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterEvent # message type for set params callback
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import cv2
import numpy as np
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class assignment(Node):
    #Robot head positions
    poses = [[-1.309,-0.130901],
             [1.0472, -0.130901],
             [0.0, -0.130901],
             [-1.309,0.785398],
             [1.0472, 0.785398],
             [0.0, 0.785398]],
    # Constructor, specifying node name for the parent class
    def __init__(self):
        super().__init__('assignment')

        self.leftSide = 0
        self.rightSide = 0
        self.forwards = 0
        self.colour_detector = 0

        #Colour values
        self.red_upper = [0,0,255]
        self.green_upper = [0,255,0]
        self.blue_upper = [255,0,0]
        self.red_lower = [0,0,0]
        self.green_lower = [0,0,0]
        self.blue_lower = [0,0,0]


        # self.lowerLimit = np.uint8(lowerLimitsParam)
        # self.upperLimit = np.uint8(upperLimitsParam)

        self.current_colour_check = 0
        self.br = CvBridge()
        # publisher for controlling the servos
        # Subscriber to retrieve laserscanner data
        
        self.publisher_ = self.create_publisher(Twist, '/servoing_cmd_vel',10)
        self.subscription = self.subscription = self.create_subscription(LaserScan,'/scan_raw',self.laser_callback,10)
        #self.subscription_image = self.create_subscription(Image, '/head_front_camera/depth_registered/image_raw',self.image_callback,10)
        self.actionClient = ActionClient(self, FollowJointTrajectory, 'head_controller/follow_joint_trajectory')
        self.head_joint_names = ["head_1_joint", "head_2_joint"] # pan, tilt

        self.br = CvBridge()     

        #Head position when program started
        self.x=0.0
        self.y=0.0

        self.i = 0

        timer_period = 0.1  # seconds
        self.array = [0,0,0,0]
        # Create a timer that will trigger calls to the method timer_callback
        # at the frequency specified above
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a counter for the purpose of the messages being sent
        self.i = 0
        

    def timer_callback(self):
        # add behaviour here to be repeated at regular intervals
        # This part determines how the robot will drive.
        test = False
        drive = Twist()
        if self.array[0] == 2:
              drive.angular.z = 0.175                       
              test = True
        elif self.array[0] == 1:
              drive.angular.z = -0.175
              test = True
        if self.array[1] == 2:
                drive.angular.z = 0.90
                test = True
        elif self.array[1] == 1:
                drive.angular.z = -0.90
                test = True
        if self.array[3] == 1:
               drive.angular.z = 0.0
               drive.linear.x = 0.0
               
               send_goal(self,1)
        else:
                linVelocity = self.array[2]/25.0*1.5 + 0.2        
        drive.linear.x = linVelocity
        
        
        # publish the message to the topic
        self.publisher_.publish(drive)
        

        # This function is used for moving the robots head.
        def send_goal(self,poses):
                        head_goal = FollowJointTrajectory.Goal()
                        
                        pose = self.poses[self.i]
                        
                        trajectory = JointTrajectory()
                        trajectory.joint_names = self.head_joint_names

                        trajectory.points.append(JointTrajectoryPoint())
                        trajectory.points[0].positions = poses

                        trajectory.points[0].velocities = [1.0,1.0]
                        trajectory.points[0].time_from_start = Duration(seconds=2.0).to_msg()


                        head_goal.trajectory = trajectory

                        self.actionClient.wait_for_server()
                        self.get_logger().info('Setting pose %s' % poses)
                        self._send_goal_future = self.actionClient.send_goal_async(head_goal, feedback_callback=self.feedback_callback) # returns rclpy.task.Future set to ClientGoalHandle
                        self._send_goal_future.add_done_callback(self.goal_response_callback)

                        # cycle through the different poses each time the timer is triggered:
                        self.i +=1
                        self.i %=3
    # TODO: rename method to better reflect topic it is subscribed to
    def laser_callback(self, msg):
        # Log output of message received
        # TODO: update fields used from pre-defined message type
        
        #val maxRange = 
        #self.get_logger().info('I heard: "%s"' % msg.data)
        
        
        #left = 0
        #front = 665/3
        #right = 665/3*2
        
        left = 0
        front = 222
        right = 493
        countRight = 0
        countForward = 0
        countLeft = 0
        rightStartCheck = 333
        rightEndCheck = 332
        drive = Twist()
        self.append = []
        corner = False
        maze = True
        #while maze == True:
        for i in range(172):
                if(not(math.isinf(msg.ranges[i]))):
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

        if self.leftSide >= 5 and self.rightSide >= 5 and self.forwards >= 2.6 and self.forwards <=4:
                        self.mazeComplete = True
                        self.array[3] = 1
                        self.array[0] = 0
                        self.array[1] = 0
                        self.array[2] = 0
                        drive.angular.z = 0.0
                        drive.linear.x = 0.0

               
                        self.actionClient.send_goal(self,1)
                        

        else:
                if(countForward > countLeft or countForward > countRight):
                        corner == True
                        
                else:
                        corner == False        
                
                if(corner == False):

                        if(self.leftSide > self.rightSide):
                                self.array[0] = 1
        
                        elif(self.rightSide > self.leftSide):
                                self.array[0] = 2
                                        
                else:
                        if(self.leftSide>self.forwards):
                                self.array[1] = 1
                
                        elif(self.rightSide>self.forwards):
                
                                self.array.append(2)
                                self.array[1] = 2
        
        #This function is used for detecting different colours.
        def colour_send_request(self,upper,lower):

                self.cli = self.create_client(SetParameters, '/colour_detector/set_parameters')
                while not self.cli.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('service not available, waiting again...')
                self.req = SetParameters.Request()
                if self.colour_detector == 1:
                        parameter = Parameter()
                        parameter.name = "lower_limits"
                        parameter.value.type = 7 # integer array value
                        parameter.value.integer_array_value = lower

                        self.req.parameters.append(parameter)

                        parameter = Parameter()
                        parameter.name = "upper_limits"
                        parameter.value.type = 7 # integer array value
                        parameter.value.integer_array_value = upper #[0,0,255]

                        self.req.parameters.append(parameter)

                        self.future = self.cli.call_async(self.req)

        #Check between two types of coloured boxes
        def colour_switcher(self):
                if self.current_colour_check == 0:
                        send_request(self,self.blue_upper,self.blue_lower)
                elif self.current_colour_check == 1:
                        send_request(self,self.green_upper,self.green_lower)
    
    def initParams(self):
        limits_description = ParameterDescriptor(description='Limits used by OpenCV colour filter, listed BGR')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('output_topic', '/colour_detection')
        self.declare_parameter('show_trackbars', True)
        self.declare_parameter('lower_limits', [0, 0, 0], limits_description)
        self.declare_parameter('upper_limits', [255, 255, 255], limits_description)
        self.lowerLimit = np.uint8([0,0,0])
        self.upperLimit = np.uint8([255,255,255])

        self.showTrackbars = self.get_parameter('show_trackbars').get_parameter_value().bool_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        lowerLimitsParam = self.get_parameter('lower_limits').get_parameter_value()._integer_array_value
        upperLimitsParam = self.get_parameter('upper_limits').get_parameter_value()._integer_array_value
        self.lowerLimit = np.uint8(lowerLimitsParam)
        self.upperLimit = np.uint8(upperLimitsParam)
        
        

        self.add_on_set_parameters_callback(self.params_callback)
    
#     def image_callback(self, imgMsg):

#         image = self.br.imgmsg_to_cv2(imgMsg)

#         # Generate a max that only inlcudes pixels from the image in the specified colour ranges
#         #mask = cv2.inRange(image, self.lowerLimit, self.upperLimit)
    
#         #Remove some of the smaller individual pixels detected as noise
#         mask = cv2.erode(mask, None, iterations=2)
#         mask = cv2.dilate(mask, None, iterations=2)

#         #overlay the mask on the original image
#         result = cv2.bitwise_and(image, image, mask=mask)

#         # find contours in the mask and initialize the current
#         # (x, y) center of the ball
        
#         contours = self.grab_contours(mask.copy())
        
#         self.x=0.0
#         self.y=0.0
#         self.radius=0.0

#         # only proceed if at least one contour was found
#         if len(contours) > 0:
#             # find the largest contour in the mask, then use
#             # it to compute the minimum enclosing circle and
#             # centroid
#             c = max(contours, key=cv2.contourArea)
#             ((self.x, self.y), self.radius) = cv2.minEnclosingCircle(c)
            
#             # only proceed if the radius meets a minimum size
#             if self.radius > 10:
#                 # draw the circle and centroid on the frame
#                 cv2.circle(result, (int(self.x), int(self.y)), int(self.radius),
#                     (0, 255, 255), 2)
#                 cv2.circle(result, (int(self.x), int(self.y)), 5, (0, 0, 255), -1) # draw small circle at centre

#                 msg = Point32()
#                 msg.x = self.x
#                 msg.y = self.y
#                 msg.z = self.radius
#                 self.publisher_.publish(msg)

        

#         cv2.imshow(self.windowName, result)
#         cv2.waitKey(1)


        

        #Triggered when goal is accepted/rejected
        def goal_response_callback(self, future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                        self.get_logger().info('Goal was rejected by server')
                return

                self.get_logger().info('Goal accepted by server, waiting for result')

                self._get_result_future = goal_handle.get_result_async()
                self._get_result_future.add_done_callback(self.get_result_callback)

        def get_result_callback(self, future):
                result = future.result().result
                if(result.error_code == 0):
                        self.get_logger().info('Goal complete successfully')
                elif(result.error_code == -1):
                        self.get_logger().error('Invalid goal: {0}'.format(result.error_string))
                else:
                        self.get_logger().error('Goal failed: {0}'.format(result.error_string))
                #rclpy.shutdown()



# Main method to be defined as entry point in setup.py
def main(args=None):

    # Initialise ROS 2 for this node
    rclpy.init(args=args)

    # Create an instance of the node subclass defined above
    assignmnet = Assignment()
    
    # Start the ROS 2 spinner with a reference to the node subclass
    rclpy.spin(assignment)

    # Destroy the node and shutdown ROS 2
    assignment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
