#!/usr/local/bin/python3
#
# to run:
# 
#
import sys
from roboclaw import Roboclaw
import time

def main():
    #
    # get the encoder counts
    #
    # address of the RoboClaw as set in Motion Studio
    #
    address = b'\x80'
    #
    # Creating the RoboClaw object, serial port and baudrate passed
    #
    robo = Roboclaw("/dev/ttymxc2", 9600) 
    #
    # Starting communication with the RoboClaw hardware
    #
    opened = robo.Open()
    if opened:
        #
        # Start motor 1 in the forward direction at half speed
        #
        counts = robo.ReadEncM1(b'\x80')
        print("motor 1 counts: ", counts)
        time.sleep(1.0)
    else:
        print("port did not open")

    return

if __name__ == "__main__":
        #
        # run main as standalone applicatiuon
        #
    main()