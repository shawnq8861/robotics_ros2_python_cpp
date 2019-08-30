#!/usr/bin/python3

import subprocess
import sys

#
# pass in a single command line argment of 4 characters
# examples:
# python3 set_cmd_vel.py 0000
# python3 set_cmd_vel.py v025
# python3 set_cmd_vel.py m001
# python3 set_cmd_vel.py m000
# python3 set_cmd_vel.py m101
# python3 set_cmd_vel.py m100
#

def main():
    #
    # error checking follows:
    #
    # must have one argument
    #
    usage_str = "usage:\npython3 set_cmd_vel.py xxxx"
    arg_count = len(sys.argv)
    if (arg_count != 2):
    	print(usage_str)
    	return
    #
    # argument must be 4 chars long
    #
    arg_str = str(sys.argv[1])
    if (len(arg_str) != 4):
    	print(usage_str)
    	return
    #
    # all good, build command string and send it
    #
    cmd_str = "echo "
    cmd_str = cmd_str + str(sys.argv[1])
    cmd_str = cmd_str + " > /dev/ttyACM0"
    print(cmd_str)
    subprocess.call(cmd_str, shell=True)

if __name__ == "__main__":
        #
        # run main as standalone applicatiuon
        #
    main()
