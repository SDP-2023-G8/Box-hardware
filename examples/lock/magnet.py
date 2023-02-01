#!/usr/bin/env python3
from motors import Motors 
from time import time, sleep 

magnet = Motors() 
port = 0         
speed = 100          
#run_time = 2         

 
#magnet.move_motor(port,speed)      
#start_time = time() 

#while time() < start_time + run_time:
#	pass
 
#magnet.stop_motors() 
def lock(){
	magnet.move_motor(port,speed)      
}

def unlock(){
	magnet.stop_motors() 
}
