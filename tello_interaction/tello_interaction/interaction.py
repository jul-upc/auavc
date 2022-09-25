from tello_msgs.srv import TelloAction
from tello_msgs.msg import TelloResponse

import rclpy
from rclpy.node import Node


class TelloInteraction(Node):

    def __init__(self):
        super().__init__('tello_interaction') #Node initialization
         
        #Declare client to service tello_action
        self.cli = self.create_client(
            TelloAction,
            'tello_action')

        #wait until the service is exposed 
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #subscribe to the tello_response topic registering the listener_callback function to answer
        self.subscription = self.create_subscription(
            TelloResponse,
            'tello_response',
            self.listener_callback,
            10)

        #Flag to be turn on when waiting for either take-off / land triggers so that 
        self.waiting_response = False
    
    def take_control(self):
        # init the autonomous flight by taking off

        self.get_logger().info('Sending takeoff request...')

        #Fill the rquest srv and publish it
        req = TelloAction.Request()
        req.cmd = "takeoff"
        self.future = self.cli.call_async(req) #Self future contains the acknowledgement of the message sent
                
        #Set the origin to 0 to recognise that the srvs call corresponds to take off                
        self.origin = 0
        #Set the origin waiting_response to know from the tello_response topic that the msg has been executed
        self.waiting_response = True

        return None


    def land(self):

        self.get_logger().info('Sending land request:')

        #Fill the rquest srv and publish it
        req = TelloAction.Request()
        req.cmd = "land"
        self.future = self.cli.call_async(req)

        #Set the origin to 0 to recognise that the srvs call corresponds to land               
        self.origin = 1
        #Set the origin waiting_response to know from the tello_response topic that the msg has been executed
        self.waiting_response = True

        return None


    def dispatch_trajectory(self):
        #Creates a timer object to call at a fixed pace the the function trajectory callback

        self.get_logger().info('Dispatching trajectory')

        timer_period = 1.0
        self.vx = 50
        self.count = 0
        self.timer = self.create_timer(timer_period, self.trajectory_callback)

        return None


    def trajectory_callback(self):

        self.get_logger().info('Sending velocity cmd num:{num}'.format(num=self.count))

        #Fill the rquest srv and publish it
        req = TelloAction.Request()
        req.cmd = 'rc {vy} {vx} {vz} {vyaw}'.format(vx = self.vx, vy=0, vz = 0, vyaw=0)
        print('[Hey:] req.cmd')

        self.cli.call_async(req)

        # Land after condition
        if self.count >10:

            self.timer.cancel() #important, cancel the timer to not act more
            self.land()
        
        self.vx = -self.vx
        self.count = self.count +1

        return None 

    def listener_callback(self, msg):
        # callbact to the tello_response topic to know when cmds (other than rc) have been executed
        # This function orquestrate the node behaviour
        
        if (self.waiting_response and (self.future is not None)): # Only works after takeoff or land calls

            self.waiting_response = False

            if self.origin==0: # from take off
                if msg.rc == 1: # rc = 1 means well executed
                    self.dispatch_trajectory()
                else:
                    self.take_control() #retrying           
            
            else:
                if msg.rc == 1:  # rc = 1 means well executed
                    return None
                else:
                    self.land() #retrying

        return None

        
def main(args=None):

    # init python client
    rclpy.init(args=args) 

    # instantiate the node
    interaction = TelloInteraction()

    # init the flight
    interaction.take_control()

    # Activate the spin that keeps alive the system and takes care of the callbacks
    rclpy.spin(interaction) # blocks the program

    # destructors
    interaction.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()