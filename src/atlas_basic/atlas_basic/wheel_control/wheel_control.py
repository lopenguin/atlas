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
import math
import rclpy
from rclpy.node import Node

from .encoder import Encoder
from .driver_replacement import Driver
from .gyro import Gyro
from sensor_msgs.msg import JointState

## CONSTANTS
LOW_LEVEL_RATE = 100 # Hz
ROS_SEND_RATE = 75 # Hz

ENC_TO_RAD = 1.0/(16 * 45) * 2 * math.pi # rad / enc rev
VEL_TIME_CONST =  LOW_LEVEL_RATE / 5.0 # Hz
CMD_TIME_CONST = LOW_LEVEL_RATE / 40.0 # Hz
POS_TIME_CONST = 20 / LOW_LEVEL_RATE # s

PWM_SLOPE_L = 9.03114 # PWM / (rad/s)
START_INCPT_L = 55 # PWM val
PWM_SLOPE_R = 9.064  # PWM / (rad/s)
START_INCPT_R = 55 # PWM val


#
#   Encoder angular speed to global rad/s
#
def encToGlobal(enctdot_L, enctdot_R):
    ## Vehicle Constants
    R = 33.0 # mm
    d2 = 137.0 # mm (wheel-to-wheel spacing)

    vx = enctdot_L*R/2   + enctdot_R*R/2
    wz = -enctdot_L*R/d2 + enctdot_R*R/d2

    return (vx, wz)
    

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


        # Subscription variables
        self.cmdvel = [0, 0]
        self.cmdtime = self.get_clock().now()

        # Timer variables
        self.last_time = self.get_clock().now()
        self.last_theta_L = 0.0
        self.last_thetadot_L = 0.0
        self.last_theta_R = 0.0
        self.last_thetadot_R = 0.0
        self.last_radz = 0.0

        self.lastdesvel = [0, 0]
        self.lastdespos = [0, 0]
        self.desmsg = JointState()
        self.actmsg = JointState()
        self.newmsg = False

        # Low-level timer
        dt = 1.0/LOW_LEVEL_RATE
        self.timer = self.create_timer(dt, self.cb_timer)

        # Publishing timer
        dtpub = 1.0/ROS_SEND_RATE
        self.pubtimer = self.create_timer(dtpub, self.cb_pubtimer)

    
    # Timer callback at LOW_LEVEL_RATE
    def cb_timer(self):
        # note current time
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1000000000.
        self.last_time = now


        ## Process encoders and convert to wheel angles!
        # get encoder readings
        theta_L = self.encoder.getLeft()*ENC_TO_RAD
        theta_R = self.encoder.getRight()*ENC_TO_RAD
        # Get derivatives
        thetadot_L = (theta_L - self.last_theta_L) / dt
        thetadot_R = (theta_R - self.last_theta_R) / dt
        # Filter derivatives
        thetadot_L = self.last_thetadot_L + VEL_TIME_CONST*dt*(thetadot_L - self.last_thetadot_L)
        thetadot_R = self.last_thetadot_R + VEL_TIME_CONST*dt*(thetadot_R - self.last_thetadot_R)
        # update last values
        self.last_theta_L = theta_L
        self.last_thetadot_L = thetadot_L
        self.last_theta_R = theta_R
        self.last_thetadot_R = thetadot_R


        ## Process the Gyro
        (radzdot, sat) = self.gyro.read()  # rad/s
        radz = self.last_radz
        if not sat:
            radz = dt*radzdot + self.last_radz   # rad
            # update last radz
            self.last_radz = radz

        ## Process encoder for theta dot
        enc_radzdot = encToGlobal(thetadot_L, thetadot_R)
        enc_radzdot = enc_radzdot[1]

        ## Process the commands.
        # only run if command is recent
        cmdPWM = [0.0, 0.0]
        desvel = [0.0, 0.0]
        despos = self.lastdespos
        if ((now - self.cmdtime).nanoseconds / 1000000000 <= 1.0):
            # Filter cmd vel
            desvel[0] = self.lastdesvel[0] + CMD_TIME_CONST*dt*(self.cmdvel[0] - self.lastdesvel[0])
            desvel[1] = self.lastdesvel[1] + CMD_TIME_CONST*dt*(self.cmdvel[1] - self.lastdesvel[1])

            # Euler integrate to get despos
            despos = [desvel[0]*dt + self.lastdespos[0], desvel[1]*dt + self.lastdespos[1]]
            self.lastdespos = despos

            # Add corrective factor for position offset
            vcorrect = [0, 0]
            vcorrect[0] = (despos[0] - theta_L)/POS_TIME_CONST
            vcorrect[1] = (despos[1] - theta_R)/POS_TIME_CONST

            # Generate motor commands (convert wheel speed to PWM)
            cmdPWM[0] = START_INCPT_L*(self.cmdvel[0]!=0) + PWM_SLOPE_L*(desvel[0] + vcorrect[0])
            cmdPWM[1] = START_INCPT_R*(self.cmdvel[1]!=0) + PWM_SLOPE_R*(desvel[1] + vcorrect[1])

            cmdPWM[0] = float(math.floor(cmdPWM[0]))
            cmdPWM[1] = float(math.floor(cmdPWM[1]))

            # establish limits
            if (cmdPWM[0] > 255):
                cmdPWM[0] = 254.
            if (cmdPWM[0] < -255):
                cmdPWM[0] = -254.

            if (cmdPWM[1] > 255):
                cmdPWM[1] = 254.
            if (cmdPWM[1] < -255):
                cmdPWM[1] = -254.

            # update last values
            self.lastdesvel = desvel


        # Command the wheels
        self.driver.left(cmdPWM[0])
        self.driver.right(cmdPWM[1])


        self.newmsg = False

        # Publish the actual wheel state /wheel_state
        self.actmsg = JointState()
        self.actmsg.header.stamp = now.to_msg()
        self.actmsg.name         = ['leftwheel', 'rightwheel', 'gyro']
        self.actmsg.position     = [theta_L, theta_R, radz]
        self.actmsg.velocity     = [thetadot_L, thetadot_R, radzdot]
        self.actmsg.effort       = [cmdPWM[0], cmdPWM[1], 0.0]

        # Publish the desired wheel state /wheel_desired
        self.desmsg = JointState()
        self.desmsg.header.stamp = now.to_msg()
        self.desmsg.name         = ['leftwheel', 'rightwheel']
        self.desmsg.position     = [despos[0], despos[1]]
        self.desmsg.velocity     = [desvel[0], desvel[1]]
        self.desmsg.effort       = [cmdPWM[0], cmdPWM[1]]

        self.newmsg = True
        



    # Timer callback at ROS_SEND_RATE
    # publishes to /wheel_desired and /wheel_state
    def cb_pubtimer(self):
        if (self.newmsg):
            # Publish the actual wheel state /wheel_state
            self.pub_actual.publish(self.actmsg)

            # Publish the desired wheel state /wheel_desired
            self.pub_desired.publish(self.desmsg)

            self.newmsg = False

    # Subscription callback
    # /wheel_command
    def cb_cmd(self, msg):
        self.cmdvel = msg.velocity
        self.cmdtime = self.get_clock().now()




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