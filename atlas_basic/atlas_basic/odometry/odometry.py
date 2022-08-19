'''
odometry.py

   Odometry node.  This
   (a) converts both a body velocity command to wheel velocity commands.
   (b) estimates the body velocity and pose from the wheel motions
       and the gyroscope.

   Node:       /odometry
   Publish:    /odom                   nav_msgs/Odometry
               TF odom -> base         geometry_msgs/TransformStamped
               /wheel_command          sensor_msgs/JointState
   Subscribe:  /vel_cmd                geometry_msgs/Twist
               /wheel_state            sensor_msgs/JointState

Cowritten by Tyler Nguyen and Lorenzo Shaikewitz for ME 169
'''

import math
import tf2_ros
import rclpy
from rclpy.node import Node
import rclpy.time

from geometry_msgs.msg  import Point, Quaternion, Twist, Vector3
from geometry_msgs.msg  import TransformStamped, Vector3
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import JointState


#
#   Constants
#
R = 33.0 / 1000   # Wheel radius (m)
d = 68.5 / 1000  # Halfwidth between wheels (m)


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # variable to save last wheel state time
        self.last_wheelstate_time = self.get_clock().now()

        # Publishers (TODO: Check QoS)
        self.pub_wheelcmd = self.create_publisher(\
            JointState  , '/wheel_command', 3)
        self.pub_odom = self.create_publisher(\
            Odometry    , '/odom', 10)

        # Create a TF2 transform broadcaster.
        self.tfpub_odomtobase = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.sub_velcmd = self.create_subscription(\
            Twist     , '/vel_cmd', self.cb_velcmd, 1)
        self.sub_wheelstate = self.create_subscription(\
            JointState, '/wheel_state', self.cb_wheelstate, 1)

    # Callback for /vel_cmd
    def cb_velcmd(self, msg):
         # Grab the forward and spin (velocity) commands.
        vx_cmd = msg.linear.x
        wz_cmd = msg.angular.z

        # CONVERT THE BODY VELOCITY COMMANDS TO L/R WHEEL COMMANDS
        lpsi_dot = vx_cmd*1/R - wz_cmd*d/R
        rpsi_dot = vx_cmd*1/R + wz_cmd*d/R

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = ['lwheel', 'rwheel']
        msg.velocity     = [lpsi_dot,    rpsi_dot    ]
        self.pub_wheelcmd.publish(msg)

    # Callback for /wheel_state
    def cb_wheelstate(self, msg):
        ## Grab the timestamp, wheel and gyro position/velocities.
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp) 
        dt = (timestamp - self.last_wheelstate_time).nanoseconds / 1000000000.
        self.last_wheelstate_time = timestamp

        lpsi = msg.position[msg.name.index('lwheel')]
        lpsi_dot = msg.velocity[msg.name.index('lwheel')]

        rpsi = msg.position[msg.name.index('rwheel')]
        rpsi_dot = msg.velocity[msg.name.index('rwheel')]

        gyro_radz = msg.position[msg.name.index('gyro')]
        gyro_radzdot = msg.velocity[msg.name.index('gyro')]


        ## Compute vel and orientation using encoders
        vx = R/2     * (rpsi_dot + lpsi_dot)
        # wz = R/(2*d) * (rpsi_dot - lpsi_dot)
        wz = gyro_radzdot


        ## define intermediates: position and angle change
        dp     = vx * dt
        dtheta = wz * dt


        # Update the pose.
        self.x     += dp * math.cos(self.theta + dtheta/2)
        self.y     += dp * math.sin(self.theta + dtheta/2)
        # self.theta += dt * wz
        self.theta = gyro_radz

        # Convert to a ROS Point, Quaternion, Twist (lin&ang veloocity).
        p = Point(x=self.x, y=self.y, z=0.0)
        q = Quaternion(w=math.cos(self.theta/2), x=0.0, y=0.0, z=math.sin(self.theta/2))
        t = Twist(linear=Vector3(x=vx, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=wz))

        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp            = timestamp.to_msg()
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.pose.pose.position      = p
        msg.pose.pose.orientation   = q
        msg.twist.twist             = t
        self.pub_odom.publish(msg)

        # Create the transform msg and broadcast (reuse the time stamp).
        msg = TransformStamped()
        msg.header.stamp            = timestamp.to_msg()
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.transform.translation   = Vector3(x=p.x, y=p.y, z=p.z)
        msg.transform.rotation      = q
        self.tfpub_odomtobase.sendTransform(msg)


def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = OdometryNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()