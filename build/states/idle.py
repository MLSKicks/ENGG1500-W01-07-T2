from requests import request
from state import State
import time

class Idle(State):
    def run(self):
        start = time.time()
        n = 5
        while elapsed_time < n:
            # TODO Implement PID Control for given time
            elapsed_time = time.time() - start
            # TODO Change to a different state and break out of this entire function
        
        lduty, rduty = 0, 0
        # TODO Stop all sensor's power
        
        while True:
            self.request_help()
            time.sleep(10)
            

    def next(self):
        pass
    
    def request_help(self):
        print("Help")