#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv  import SetBool
# import smbus
import os
import time
import numpy as np
import smbus2 as smbus

class MotorControl:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = 0x04

    def setMotor(self, id, speed):
        """
        Mode 2 is Forward.
        Mode 3 is Backwards.
        """
        direction = 2 if speed >= 0 else 3
        speed = np.clip(abs(speed), 0, 100)
        byte1 = id << 5 | 24 | direction << 1
        byte2 = int(speed * 2.55)
        self.__write_block([byte1, byte2])

    def stopMotor(self, id):
        """
        Mode 0 floats the motor.
        """
        direction = 0
        byte1 = id << 5 | 16 | direction << 1
        self.__write(byte1)

    def stopMotors(self):
        """
        The motor board stops all motors if bit 0 is high.
        """
        print('[INFO] [MotorControl] Stopping all motors...')
        self.__write(0x01)

    def __write(self, value):
        try:
            self.bus.write_byte_data(self.address, 0x00, value)
        except IOError as e:
            print('I/O error({0}): {1}'.format(e.errno, e.strerror))

    def __write_block(self, values):
        try:
            msg = smbus.i2c_msg.write(self.address, values)
            self.bus.i2c_rdwr(msg)
        except IOError as e:
            print('I/O error({0}): {1}'.format(e.errno, e.strerror))

class Motors(object):
	def __init__(self):
		print ("Starting SMBus . . .")
		self.bus = smbus.SMBus(1)
		time.sleep(2)
		print ("SMBus Started.")
		self.mc = MotorControl()
		self.encoder_address = 0x05
		self.encoder_register = 0x0
		self.num_encoder_ports = 6
		self.refresh_rate = 10 #refresh rate - reduces errors in i2c reading

	def move_motor(self,id,speed):
		self.mc.setMotor(id, speed)

	def stop_motor(self,id):
		self.mc.stopMotor(id)

	def stop_motors(self):
		self.mc.stopMotors()

	def __i2c_read_encoder(self):
		self.encoder_data = self.bus.read_i2c_block_data(self.encoder_address, 	\
							self.encoder_register, 	\
							self.num_encoder_ports)

magnet = Motors() 
port = 3         
speed = 255         

 
def lock():
	magnet.move_motor(port,speed)      


def unlock():
	magnet.stop_motors() 
        

class LockService(Node):

    def __init__(self):
        super().__init__('lock_service')
        self.srv = self.create_service(SetBool, '~/lock', self.callback)

    def callback(self, request, response):
        if (request.data == True):
            lock()
            self.get_logger().info('door lock')
        else:
            unlock()
            self.get_logger().info('door unlock')
        response.success = True
        return response


def main():
    rclpy.init()
    lock_service = LockService()
    rclpy.spin(lock_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




