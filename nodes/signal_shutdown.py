#!/usr/bin/env python
# Simple script for shutting down the raspberry Pi at the press of a button.
# by Inderpreet Singh
# Adapted by Jake Rye and Bob Davis

import RPi.GPIO as GPIO
import time
import os
import logging
import rospy
from std_msgs.msg import String


def setup_gpio_pins():
    # Use the Broadcom SOC Pin numbers
    # Setup the Pin with Internal pullups enabled and PIN in reading mode.
    logger.debug('Setting GPIO27 / PIN13 to INPUT PULLUP')
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(27, GPIO.IN, pull_up_down = GPIO.PUD_UP) # PIN13 / YEL / SW-COM

    # Turn delay relay on
    logger.debug('Setting GPIO22 / PIN15 to OUTPUT HIGH')
    GPIO.setup(22, GPIO.OUT) # PIN15 / WHT / CH1
    GPIO.output(22, GPIO.HIGH)

    # Send a stable low signal to the power switch
    logger.debug('Setting GPIO17 / PIN11 to OUTPUT LOW')
    GPIO.setup(17, GPIO.OUT) # PIN15 / WHT / CH1
    GPIO.output(17, GPIO.LOW)


def check_for_shutdown():
    # Monitor pin for stable signal to safely shutdown
    while not rospy.is_shutdown():
        if not GPIO.input(27):
            logger.debug('Initiating safe shutdown sequence')
            successful_debounce = True
            for i in range(5000):
                if GPIO.input(27):
                    logger.debug('Signal interrupted, breaking out of safe shutdown sequence')
                    successful_debounce = False
                    break
                time.sleep(0.001)
            if successful_debounce:
                logger.debug('Safely shutting down')
                rospy.signal_shutdown("Safely shutting down due to Power Off Button")
                time.sleep(5)
                os.system("sudo shutdown -r now")
                break
        rospy.sleep(1.)


if __name__ == '__main__':
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    rospy.init_node('signal_shutdown')
    setup_gpio_pins()
    try:
        check_for_shutdown()
    except rospy.ROSInterruptException:
        pass
