import subprocess
from recovery_pathfinder.recoveryStatus import RECOVERYSTATUS 


class RSSI():
    def __init__(self, node):
        self.node           = node
        self.signalThresh   = -70      # dbm
        self.timer          = self.node.create_timer(5.0, self.getSignalLevel)
        self.interface      = 'wlp3s0' # Replace with your wireless interface name
        self.signalAcquired = False

    def getSignalLevel(self):
        self.signalAcquired = False
        try:
            print("REcoveryMode: ",self.node.RECOVERYMODE)
            # Run iwconfig command to get wireless interface information
            result = subprocess.run(['iwconfig', self.interface], capture_output=True, text=True, check=True)
            output_lines = result.stdout.split('\n')
            for line in output_lines:
                if 'Signal level' in line:
                    signal_level = line.split('=')[-1].split()[0]  # Extract the signal level value
                    self.node.logger.info(f"signal strength: {signal_level}")
                    self.updateRecoveryMode(int(signal_level))
                    self.signalAcquired = True
                    
            if(not self.signalAcquired):   
                self.node.RECOVERYMODE = RECOVERYSTATUS.RECOVERY

           
        except subprocess.CalledProcessError as e:
            self.node.logger.warn(f"Error: {e}")
            self.node.RECOVERYMODE = RECOVERYSTATUS.RECOVERY
           

    def updateRecoveryMode(self, signalLevel):
        if((signalLevel < self.signalThresh) and (self.node.RECOVERYMODE == RECOVERYSTATUS.CHECKPULSE)):
            self.node.RECOVERYMODE = RECOVERYSTATUS.RECOVERY
            self.node.logger.warn("Disconnected from the network")
    

        

