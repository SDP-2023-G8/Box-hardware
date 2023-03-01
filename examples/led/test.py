import RPi.GPIO as GPIO
from time import sleep 
PIN = 14
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(PIN,GPIO.OUT)

# While loop
while True:
        GPIO.output(PIN,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(PIN,GPIO.LOW)
        time.sleep(1)
