import magnet
from time import time, sleep
runtime = 10

magnet.lock()

start_time = time() 
while time() < start_time + run_time:
   pass

magnet.unlock()

