'''
Saketh Ayyagari

Node publishing odometry data (specifically robot's pose, which consists of
x, y, and theta values)
'''

#!usr/bin/env python3
import math
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
'''
Uses velocity values from JointState message and publishes new Odometry message
'''
class OdometryPublisher(Node): 
    def __init__(self):
        super().__init__('odometry_publisher') # intiializes node name
        # constants
        self.wheel_separation = 0.137 # number of meters between wheels
        self.wheel_radius = 0.033 # radius of wheels in 
        
        # initializing pose (x, y, theta), velocity, and time variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = 0
        s
        # subscribing to the joint_states topic to get velocities of each wheel
        # joint_states_callback() is run everytime the node receives a message
        self.create_subscription(JointState, 'joint_states', 
                            self.joint_states_callback, 10)

        # publishing to an odometry topic
        odometry_publisher = self.create_publisher(Odometry, 'robot_odometry', 10)

    # TODO: simplify joint_states_callback() into smaller functions
    '''
    Gets wheel velocities (rad/s) from a JointStates message 
    '''
    def get_wheel_velocities(self, message):
        left_angular_velocity = message.velocity[message.index("left_wheel_joint")]
        right_angular_velocity = message.velocity[message.index("right_wheel_joint")]

        return left_angular_velocity, right_angular_velocity

    '''
    Updates change in time since last called
    '''
    def get_delta_time(self):
        current_time = self.get_clock().now()
        delta_time = current_time - self.prev_time
        self.prev_time = current_time
        
        return delta_time
    '''
    Updates pose (x, y, theta) global variables given velocity values
    '''
    def update_pose(self, linear_velocity, angular_velocity):
        delta_time = self.get_delta_time()
        # integrates velocity values and adds them to current parts of pose
        self.theta += self.angular_vel * self.delta_time # estimating heading w/ respect to initial heading
        # updates x and y based on component values of velocity
        self.x += self.linear_vel * math.cos(self.theta) * self.delta_time 
        self.y += self.linear_vel * math.sin(self.theta) * self.delta_time

    def joint_states_callback(self, joint_states):
        
        # gets angular velocity values from join_state message
        left_angular_velocity, right_angular_velocity = self.get_wheel_velocities(joint_states)

        # conversion from angular to linear velocities for each wheel
        left_wheel_vel = self.wheel_radius * left_angular_velocity
        right_wheel_vel = self.wheel_radius * right_angular_velocity

        # using odometry equations to get total linear and angular velocities
        '''
        Equations
        lin_vel = (v_left + v_right) / 2
        ang_vel = (v_right - v_left) / wheel_seperation
        '''
        self.linear_vel = (left_wheel_vel + right_wheel_vel) / 2
        self.angular_vel = (right_wheel_vel - left_wheel_vel) / self.wheel_seperation
        # using velocity values to estimate pose
        self.update_pose(self.linear_vel, self.angular_vel)
        
        # creating new odometry message  
        odom_message = Odometry()
        odom_message.child_frame_id = "base_link"
        odom_message.header.frame_id = "odom"
        odom_message.header.stamp = self.get_clock().now()

        # assigning pose values to odometry message
        odom_message.pose.pose.position.x = self.x
        odom_message.pose.pose.position.y = self.y
        odom_message.pose.pose.position.z = 0
    
        # converting euler orientation to Quaternion orientation
        orientation_quat = quaternion_from_euler(0, 0, self.theta, 0)
        odom_message.pose.pose.orientation = Quaternion(x=orientation_quat[0], y=orientation_quat[0], z=self.theta, )
        
        # publishes odometry message to /robot_odometry topic
        self.odometry_publisher.publish(odom_message)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass
    odometry_publisher.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()