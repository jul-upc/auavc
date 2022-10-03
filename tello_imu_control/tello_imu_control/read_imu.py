import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter


import serial
import math


class readImu(Node):

    def __init__(self):
        #Initialize the node
        super().__init__('read_imu')

        #Declare the publisher. Note the topic name vel_imu
        self.publisher_ = self.create_publisher(Twist, 'vel_imu', 10) #publish at cmd_vel topic

        #Declare parameters with default value
        self.declare_parameter('per_axis_v_max', 2.0)
        self.declare_parameter('pub_freq', 20.0)
        self.declare_parameter('x_vel_norm',0.1)
        self.declare_parameter('serial_port','/dev/ttyACM0')        

        #Get values of parameters. The obtained value is different from the standard ones if defined

        pub_freq = self.get_parameter('pub_freq').value
        self.vmax = self.get_parameter('per_axis_v_max').value
        self.x_vel_norm = self.get_parameter('x_vel_norm').value
        self.serial_port = self.get_parameter('serial_port').value

        self.get_logger().info("freq:%s, vmax:%s, x_vel_norm:%s, serial_port:%s" %
                        (str(pub_freq),
                        str(self.vmax),
                        str(self.x_vel_norm),
                         self.serial_port,))


        #Set timer freq and register the callback
        timer_period = 1.0/pub_freq  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        roll, pitch = self.read_serial()

        #Fill the msg and publish
        twist_msg = Twist()
        twist_msg.linear.x = self.x_vel_norm*self.vmax
        twist_msg.linear.y = math.sin(roll)*self.vmax
        twist_msg.linear.z = math.sin(pitch)*self.vmax
        self.publisher_.publish(twist_msg)

    def read_serial(self):

        baud = 10000
        timeout = 2

        with serial.Serial(self.serial_port, baud, timeout=timeout) as ser:

            eol = b'>'
            leneol = len(eol)
            line = bytearray()
            while True:
                c = ser.read(1)
                line += c
                if line[-leneol:] == eol:
                    break
            
            try:
                readings = str(line.decode("utf-8")[1:-2]).split(',')
                pitch = float(readings[0])*math.pi/180
                roll = float(readings[1])*math.pi/180

            except:

                pitch = None
                roll = None

            return (roll, pitch)



def main(args=None):
    rclpy.init(args=args)

    read_imu = readImu()

    rclpy.spin(read_imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()