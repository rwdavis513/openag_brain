#!/usr/bin/env python
# Simple script for shutting down the raspberry Pi at the press of a button.
# by Inderpreet Singh
# Adapted by Jake Rye and Bob Davis

import RPi.GPIO as GPIO
import time
import os
import logging
import rospy
from std_msgs.msg import Float64


def setup_gpio_pins():
    # Use the Broadcom SOC Pin numbers
    # Setup the Pin with Internal pullups enabled and PIN in reading mode.
    logger.debug('Setting GPIO27 / PIN13 to INPUT PULLUP')
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(13, GPIO.IN, pull_up_down = GPIO.PUD_UP) # PIN13 / BCM 27 YELLOW / SW-COM

    # Turn delay relay on
    logger.debug('Setting GPIO22 / PIN15 to OUTPUT HIGH')
    GPIO.setup(15, GPIO.OUT) # PIN15 / BCM 22 / WHITE / CH1
    GPIO.output(15, GPIO.HIGH)


def check_for_shutdown(pub13):
    # Monitor pin for stable signal to safely shutdown
    while not rospy.is_shutdown():
        pub13.publish(GPIO.input(13))
        if not GPIO.input(13):
            logger.debug('Initiating safe shutdown sequence')
            successful_debounce = True
            for i in range(5000):
                if GPIO.input(13):
                    logger.debug('Signal interrupted, breaking out of safe shutdown sequence')
                    successful_debounce = False
                    break
                time.sleep(0.001)
            if successful_debounce:
                logger.debug('Safely shutting down')
                rospy.signal_shutdown("Safely shutting down due to Power Off Button")
                time.sleep(5)
                GPIO.output(15, GPIO.LOW)
                os.system("sudo shutdown now")
                break
        rospy.sleep(1.)


if __name__ == '__main__':
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    rospy.init_node('signal_shutdown')
    pub13 = rospy.Publisher("/GPIO/13", Float64, queue_size=10)
    setup_gpio_pins()
    try:
        check_for_shutdown(pub13)
    except rospy.ROSInterruptException:
        pass
