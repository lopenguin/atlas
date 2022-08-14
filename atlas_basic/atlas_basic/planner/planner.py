'''
planner.py

    Planning node. 
    Converts current position and desired position into velocity/twist

   Node:       /planner
   Publish:    /vel_cmd                geometry_msgs/Twist
   Subscribe:  /odom                   nav_msgs/Odometry
               /goal_pose              geometry_msgs/PoseStamped

Cowritten by Tyler Nguyen and Lorenzo Shaikewitz for ME 169 (2022)
'''
import math
import rclpy
from rclpy.node import Node
import rclpy.time

from geometry_msgs.msg  import Pose, PoseStamped, Twist
from nav_msgs.msg       import Odometry

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner')
        # Set up class variables
        self.goal_pose = Pose()

        # Publisher
        self.pub_velcmd = self.create_publisher(\
            Twist, '/vel_cmd', 10)

        # Subscribers
        self.sub_odom = self.create_subscription(\
            Odometry, '/odom', self.cb_odom, 5)
        self.sub_goal = self.create_subscription(\
            PoseStamped, '/goal_pose', self.cb_goal, 1)

    # Callback for /odom
    def cb_odom(self, msg):
        curr_pose = msg.pose.pose
        msg_velcmd = velocityToGoal(curr_pose, self.goal_pose)
        self.pub_velcmd.publish(msg_velcmd)

    # Callback for /goal_pose
    def cb_goal(self, msg):
        # Just save the pose for odom updating
        self.goal_pose = msg.pose
        goal_t = 2*math.atan2(msg.pose.orientation.z, \
                              msg.pose.orientation.w)
        self.get_logger().info(f"Goal command received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), theta={goal_t:.2f} rad in frame '{msg.header.frame_id}'.")


# Constants for goto function
CLOSE_ENOUGH_DISTANCE = 0.02 # m
CLOSE_ENOUGH_THETA = 0.2 # rad
OMEGA_P_CONST_SPIN = 1.0
MAX_VEL = 0.6 # m/s
VEL_TIME_CONST = 2.0 # m/s
OMEGA_CONST = 0.5
GO_FORWARD_THETA = 0.75 # rad

# Computes angular/linear velocity to reach target
def velocityToGoal(curr_pose, goal_pose):
    # precompute travel distance
    dx = goal_pose.position.x - curr_pose.position.x
    dy = goal_pose.position.y - curr_pose.position.y
    d = math.sqrt(dx*dx + dy*dy)    # distance to travel
    targett = math.atan2(dy, dx)    # angle to face target
    # precompute planar angles
    curr_t = 2*math.atan2(curr_pose.orientation.z, \
                          curr_pose.orientation.w)
    goal_t = 2*math.atan2(goal_pose.orientation.z, \
                          goal_pose.orientation.w)
                          
    # initialize velocities to zero
    vx = 0.
    wz = 0.
    
    # vx: move if far away and kinda facing target
    ttogo = angleDiff(targett, curr_t)
    if (d > CLOSE_ENOUGH_DISTANCE) and (abs(ttogo) < GO_FORWARD_THETA):
        vx = clamp((1/VEL_TIME_CONST)*d*math.cos(ttogo), 0, MAX_VEL)

    # wz: move if (a) far away and not facing target or (b) close and not facing desired
    relt = angleDiff(goal_t, curr_t)
    if (d > CLOSE_ENOUGH_DISTANCE) and (abs(ttogo) > CLOSE_ENOUGH_THETA):
        wz = OMEGA_CONST*ttogo
    elif (d < CLOSE_ENOUGH_DISTANCE) and (abs(relt) > CLOSE_ENOUGH_THETA):
        wz = OMEGA_P_CONST_SPIN*relt

    # compile into message
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = wz
    return msg

# Compute the angle difference
def angleDiff(t1, t2):
    return (t1-t2) - 2.0*math.pi * round(0.5*(t1-t2)/math.pi)

# Clamp a variable between a maximum and minimum (Saturated)
def clamp(x, min_x, max_x):
    return max(min(max_x, x), min_x)



def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = PlannerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()