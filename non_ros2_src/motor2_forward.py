#!/usr/local/bin/python3
#
# to run:
# 
#
import sys
from roboclaw_3 import Roboclaw
import time

def main():
    #
    # error checking follows:
    #
    # must have one argument
    #
    usage_str = "usage:\npython3 motor_forward.py XXX"
    arg_count = len(sys.argv)
    if (arg_count != 2):
    	print(usage_str)
    #
    # get the move speed, must be between 0 and 127
    #
    else:
        speed = int(sys.argv[1])
        #
        # address of the RoboClaw as set in Motion Studio
        #
        address = 0x80
        #
        # Creating the RoboClaw object, serial port and baudrate passed
        #
        robo = Roboclaw("/dev/ttymxc2", 38400) 
        #
        # Starting communication with the RoboClaw hardware
        #
        opened = robo.Open()
        if opened:
            #
            # Start motor 1 in the forward direction at half speed
            #
            robo.ForwardM2(address, speed)
            #
            # pause for two seconds
            #
            time.sleep(2.0)
            #
            # stop the motor
            #
            robo.ForwardM1(address, 0)
        else:
            print("port did not open")

    return

if __name__ == "__main__":
        #
        # run main as standalone applicatiuon
        #
    main()