'''
wheel_control.py

   This node does basic wheel control. Additional features include:
     - stops if no commands are received after 0.25sec
     - filter the commands to avoid spikes (high torque/current loads)
     - filters the actual velocity
     - adds feedback to perfectly achieve the velocity commands

   Node:       /wheel_control
   Publish:    /wheel_state            sensor_msgs/JointState
               /wheel_desired          sensor_msgs/JointState
   Subscribe:  /wheel_command          sensor_msgs/JointState

   Other Inputs:   Encoder Channels (GPIO)
   Other Outputs:  Motor Driver Commands (via I2C)

Cowritten by Tyler Nguyen and Lorenzo Shaikewitz for ME 169
'''
import smbus
import rclpy
from rclpy.node import Node

from encoder import Encoder
from driver_replacement import Driver
from gyro import Gyro
from sensor_msgs.msg import JointState

# Global constants
LOW_LEVEL_RATE = 100 # Hz
ROS_SEND_RATE = 50 # Hz

class WheelControl(Node):

    def __init__(self, i2cbus):
        super().__init__('wheel_control')
        self.i2cbus = i2cbus

        # Low-level sensor objects
        self.encoder = Encoder()
        self.driver = Driver(i2cbus)
        self.gyro = Gyro(i2cbus)

        # Publishers (TODO: Check QoS)
        self.pub_desired = self.create_publisher(\
            JointState, '/wheel_desired', 10)
        self.pub_actual  = self.create_publisher(\
            JointState, '/wheel_state'  , 10)

        # Subscribers
        self.sub_cmd = self.create_subscription(\
            JointState, '/wheel_command', self.cb_cmd, 10)


        # Low-level timer
        dt = 1.0/LOW_LEVEL_RATE
        self.timer = self.create_timer(dt, self.cb_timer)

        # Publishing timer
        dtpub = 1.0/ROS_SEND_RATE
        self.pubtimer = self.create_timer(dtpub, self.cb_pubtimer)

    
    # Timer callback at LOW_LEVEL_RATE
    def cb_timer(self):
        pass

    # Timer callback at ROS_SEND_RATE
    # publishes to /wheel_desired and /wheel_state
    def cb_timer(self):
        pass



def main(args=None):
    # Grab the I2C bus.
    i2cbus = smbus.SMBus(1)

    # intialize ROS node.
    rclpy.init(args=args)
    node = WheelControl(i2cbus)

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()