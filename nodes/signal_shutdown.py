#!/usr/bin/env python
# Simple script for shutting down the raspberry Pi at the press of a button.
# by Inderpreet Singh
# Adapted by Jake Rye and Bob Davis in April, 2017

from periphery import GPIO
from periphery.gpio import GPIOError
import time
import os
import logging
import rospy
from std_msgs.msg import Float64

# This is needed to account for any erroraneous drops in the shutdown signal
VERIFICATION_TIME_SEC = 3          # Time to wait to verify shutdown signal
SHUTDOWN_SIGNAL_THRESHOLD = 0.98   # Percent of shutdown signals needed to
                                   # signal a shutdown (ranges from 0 to 1)

def setup_pin(pin, direction, number_of_attempts=5):
    """
    The first attempt to connect to the GPIO pins fails, but the second try succeeds. It appears to be a timing issue in the periphery library.
       It first exports to the GPIO pin to connect it and then writes the direction. Writing the direction fails, 
       because of a permission error.
    This function is a work around to try several times to allow the export function to finish creating the class in
       /sys/class/gpio/gpio{pin_number}
       http://python-periphery.readthedocs.io/en/latest/_modules/periphery/gpio.html#GPIO
    returns gpio object for writing or reading.
    """
    for _ in range(number_of_attempts):
       try:
           gpio_pin = GPIO(pin, direction)
           break
       except GPIOError as e:
           rospy.logdebug("Failed to connect to GPIO pin {}. {} attempt.".format(pin, _))
           if _+1 == number_of_attempts:
               raise e
           time.sleep(0.05)
       rospy.loginfo("GPIO Pin {} took {} tries to connect.".format(pin, _ + 1))
    return gpio_pin


def setup_gpio_pins():
    # Use the Broadcom SOC Pin numbers
    # Setup the Pin with Internal pullups enabled and PIN in reading mode.
    rospy.logdebug('Setting GPIO27 / PIN13 to INPUT PULLUP')
    # PIN13 / BCM 27 YELLOW / SW-COM - Signals Shutdown
    gpio_13 = setup_pin(13, "high")  # Pull pin high before watching if it drops low to signal shutdown
    gpio_13 = setup_pin(13, "in")  # Initializes pin to low, not sure how to set to high and watch to drop low.

    # Turn delay relay on
    # Sends signal to the relay to stay on until this drops low.
    rospy.logdebug('Setting GPIO22 / PIN15 to OUTPUT HIGH')
    gpio_15 = setup_pin(15, "out") # PIN15 / BCM 22 / WHITE / CH1
    return gpio_13, gpio_15


def check_for_shutdown(gpio_13, gpio_15):
    # Monitor pin for stable signal to safely shutdown
    while not rospy.is_shutdown():
        shutdown_signal_pin = gpio_13.read()
        rospy.loginfo(shutdown_signal_pin)
        rospy.logdebug(shutdown_signal_pin)
        if not shutdown_signal_pin:
            rospy.logdebug('Initiating safe shutdown sequence')
            successful_debounce = True
            input_sum = 0
            for i in range(VERIFICATION_TIME_SEC * 1000):
                input_sum += gpio_13.read()
                time.sleep(0.001)
            avg_signal = input_sum / ( VERIFICATION_TIME_SEC * 1000)
            rospy.logdebug('The average signal after {} seconds was {}'.format(
                           VERIFICATION_TIME_SEC, avg_signal))
            if avg_signal > (1 - SHUTDOWN_SIGNAL_THRESHOLD):  # avg_signal should be 0 for 100% valid
                rospy.logwarn('Signal interrupted, breaking out of safe shutdown sequence')
                successful_debounce = False
                continue

            if successful_debounce:
                rospy.logdebug('Safely shutting down')
                rospy.signal_shutdown('Safely shutting down due to Power Off Button')
                time.sleep(5)
                gpio_15.write(False)
                os.system("sudo shutdown -h now")
                break
        rospy.sleep(1.)


if __name__ == '__main__':
    rospy.init_node('signal_shutdown')
    gpio_13, gpio_15 = setup_gpio_pins()
    try:
        check_for_shutdown(gpio_13, gpio_15)
    except rospy.ROSInterruptException:
        pass
