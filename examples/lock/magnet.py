from motors import Motors 
from time import time, sleep 

magnet = Motors() 
port = 0         
speed = 100          

 
def lock():
	magnet.move_motor(port,speed)      


def unlock():
	magnet.stop_motors() 

