from std_msgs.msg import Header 
from recovery_pathfinder.recoveryStatus import RECOVERYSTATUS 

class pulseChecker():

    def __init__(self, node):
        self.node              = node
        self.isLost            = None
        timer_period           = 1.0     # seconds
        self.pulse_publisher   = self.node.create_publisher(Header, 'pulse', 2) 
        self.pulse_timer       = self.node.create_timer(timer_period, self.pulse_callback)
        self.pulse_check_timer = self.node.create_timer(5.0, self.pulse_check)
        
        self.robot_pulse_01_ = self.node.create_subscription(Header, '/ssugv5/pulse',  self.pulse_checker_callback, 2) # Robot fleet 01
        self.robot_pulse_02_ = self.node.create_subscription(Header, '/ssugv6/pulse',  self.pulse_checker_callback, 2)
        self.robot_pulse_03_ = self.node.create_subscription(Header, '/ssugv7/pulse',  self.pulse_checker_callback, 2)
        self.robot_pulse_04_ = self.node.create_subscription(Header, '/ssugv8/pulse',  self.pulse_checker_callback, 2)

        self.robot_pulse_05_ = self.node.create_subscription(Header, '/ssugv9/pulse',  self.pulse_checker_callback, 2) # Robot fleet 02
        self.robot_pulse_06_ = self.node.create_subscription(Header, '/ssugv10/pulse', self.pulse_checker_callback, 2)
        self.robot_pulse_07_ = self.node.create_subscription(Header, '/ssugv11/pulse', self.pulse_checker_callback, 2)
        self.robot_pulse_08_ = self.node.create_subscription(Header, '/ssugv12/pulse', self.pulse_checker_callback, 2)
       
        # prevent unused variable warning
        self.robot_pulse_01_  
        self.robot_pulse_02_  
        self.robot_pulse_03_  
        self.robot_pulse_04_  
        self.robot_pulse_06_  
        self.robot_pulse_07_  
        self.robot_pulse_08_  


    def pulse_checker_callback(self, msg):
        robot_ID = msg.frame_id
        if(self.node.init_robot_pulse[robot_ID] == None):
            self.node.init_robot_pulse[robot_ID] = msg.stamp.sec                                     # Store initial robot timestamps
        
        if(self.node.init_robot_pulse[self.node.robotName] != None):
            self.node.robot_pulse[robot_ID] = ( msg.stamp.sec - self.node.init_robot_pulse[robot_ID] ) + abs(self.node.init_robot_pulse[self.node.robotName]  - self.node.init_robot_pulse[robot_ID] )      # Remove the offset from different time values


    def pulse_check(self):
        if(self.node.RECOVERYMODE == RECOVERYSTATUS.CHECKPULSE):
            self.isLost = True

            for robot in self.node.robot_names:
                if(self.node.robotName == robot or self.node.robot_pulse[robot] == None):
                    continue

                if(abs(self.node.robot_pulse[self.node.robotName] - self.node.robot_pulse[robot]) > self.node.CONNECTION_LOST_TIME):
                    self.node.robot_pulse[robot] = None
                    self.node.logger.warn(f'Connection Lost with {robot}')

                self.isLost = False
                self.node.RECOVERYMODE = RECOVERYSTATUS.CHECKPULSE
            
            if(self.isLost):
                self.node.RECOVERYMODE = RECOVERYSTATUS.RECOVERY


    def pulse_callback(self): # Publish robots location in map pixels and timestamp as Pulse
        msg = Header()

        msg.stamp    = self.node.get_clock().now().to_msg()
        msg.frame_id = self.node.robotName

        self.pulse_publisher.publish(msg)