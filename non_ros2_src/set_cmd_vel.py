#!/usr/bin/python3

import subprocess
import sys

def main():
    #
    # build the command string
    # use arg passed in for speed
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
