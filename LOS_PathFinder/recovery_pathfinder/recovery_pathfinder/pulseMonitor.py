import rclpy
import cv2
import time
import numpy as np

from rclpy.node                             import Node     
from recovery_pathfinder.LOSpathFinder      import LOSpathFinder
from recovery_pathfinder.recoveryStatus     import RECOVERYSTATUS
from recovery_pathfinder.pulseChecker       import pulseChecker
from recovery_pathfinder.recoveryNavigator  import recoveryNavigator
from recovery_pathfinder.RSSI_monitor       import RSSI
from nav_msgs.msg                           import OccupancyGrid
from std_msgs.msg                           import Header

class PulseMonitor(Node):

    def __init__(self):
        super().__init__('pulse_monitor')

        timer_period          = 1.0     # seconds
        self.monitor_timer    = self.create_timer(timer_period, self.monitor_callback)
        self.logger           = self.get_logger()
        
        self.CONNECTION_LOST_TIME = 10.0

        self.robotName        = 'ssugv8'
        self.robot_pulse      = {}
        self.init_robot_pulse = {}
        self.robot_names      = ['ssugv5', 'ssugv6', 'ssugv7', 'ssugv8', 'ssugv9', 'ssugv10', 'ssugv11', 'ssugv12']
        self.RECOVERYMODE     = RECOVERYSTATUS.CHECKPULSE
        self.MAX_RECOVERY_ATTEMPTS = 3
        self.recoveryAttempt       = 1
        self.keyPoints             = None

        for robot in self.robot_names:
            self.robot_pulse[robot]      = None
            self.init_robot_pulse[robot] = None


        self.pulseChecker       = pulseChecker(self)       
        #self.RSSIMonitor        = RSSI(self)    
        self.LOSpathFinder      = LOSpathFinder(self)
        self.recoveryNavigator  = recoveryNavigator(self)

    def monitor_callback(self):
        if (self.RECOVERYMODE == RECOVERYSTATUS.RECOVERY):
            try:
                self.logger.info("Calculating optimum path")
                self.keyPoints = self.LOSpathFinder.getRecoveryPath()
                self.logger.info(f"Keypoints: {self.keyPoints}")


                self.RECOVERYMODE = RECOVERYSTATUS.NAVIGATE

            except Exception as err:
                print("monitor callback error: ",err)
                if (self.recoveryAttempt <= self.MAX_RECOVERY_ATTEMPTS):
                    self.logger.info(f"Starting next recovery attempt : {self.recoveryAttempt}")
                    self.recoveryAttempt += 1
                else:
                    self.logger.warn("Recovery Failed")
                    self.RECOVERYMODE = RECOVERYSTATUS.CHECKPULSE


        
        elif(self.RECOVERYMODE == RECOVERYSTATUS.NAVIGATE):
            self.recoveryNavigator.navigateToGoal(self.keyPoints)
            

def main(args=None):
    rclpy.init(args=args)
    pulse_monitor = PulseMonitor()
    rclpy.spin(pulse_monitor)

    pulse_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
