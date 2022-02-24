#!/usr/bin/env python3
import rospy
import spidev
import RPi.GPIO as GPIO
from math import pi
from sensor_custom_msgs.msg import SensorEncoder, SensorEncoderStamped

class sensor_encoder:
    def __init__(self):
        # Init node
        rospy.init_node('sensor_encoder_node', anonymous=False)
        
        # Get node name
        self.node_name = rospy.get_name()
        
        # Get ros params
        self.get_ros_params()
        
        # Init GPIO
        self.init_gpio()

        # Create topics
        self.pub_sensor = rospy.Publisher('front_encoder', SensorEncoderStamped, queue_size=1)
      
    def get_ros_params(self):
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'front_encoder')
        self.offset = rospy.get_param(self.node_name + '/offset', 0.0)
        self.frequency = rospy.get_param(self.node_name + '/frequency', 50)
        
    def init_gpio(self):
        self.spi_ch = 0
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(13, GPIO.OUT)
        GPIO.output(13, GPIO.HIGH)
        # Enable SPI
        self.spi = spidev.SpiDev(0, self.spi_ch)
        self.spi.max_speed_hz = 1000000
        
    def update(self):
        GPIO.output(13, GPIO.LOW)
        msg = [0b00000000, 0b00000000]
        recv = self.spi.xfer2(msg)
        data = recv[0]*0xFF + recv[1]
        rad = ((data / (32768 / 360)) * pi / 180) + self.offset
        GPIO.output(13, GPIO.HIGH)
        encoder = SensorEncoderStamped()
        encoder.header.stamp = rospy.Time.now()
        encoder.header.frame_id = self.frame_id
        encoder.sensor.position = rad
        encoder.sensor.offset = self.offset
        self.pub_sensor.publish(encoder)
      
    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
      
if __name__ == '__main__':
    encoder = sensor_encoder()
    try:
      encoder.run()
    except rospy.ROSInterruptException():
        pass