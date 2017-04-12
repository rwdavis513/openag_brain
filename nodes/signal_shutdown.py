#!/bin/python
# Simple script for shutting down the raspberry Pi at the press of a button.
# by Inderpreet Singh

import RPi.GPIO as GPIO
import time
import os
import logging
import rospy


def setup_gpio_pins():
    # Use the Broadcom SOC Pin numbers
    # Setup the Pin with Internal pullups enabled and PIN in reading mode.
    logger.debug('Setting GPIO27 / PIN13 to INPUT PULLUP')
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(13, GPIO.IN, pull_up_down = GPIO.PUD_UP) # PIN13 / BOARD 27 YEL / SW-COM

    # Turn delay relay on
    logger.debug('Setting GPIO22 / PIN15 to OUTPUT HIGH')
    GPIO.setup(15, GPIO.OUT) # PIN15 / BOARD 22 / WHT / CH1
    GPIO.output(15, GPIO.HIGH)


def check_for_shutdown():
    rate = rospy.Rate(10) # 10hz
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
                GPIO.output(15, GPIO.LOW)
                os.system("sudo shutdown -r now")
                break
        rospy.Rate(10)


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
