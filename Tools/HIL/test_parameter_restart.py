#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import sys

def do_parameter_restart_test(port, baudrate):
    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, timeout=0.1)

    # run command
    timeout_start = time.time()
    timeout = 10  # 10 seconds

    # clear
    ser.write("\n".encode("ascii"))
    ser.flush()
    ser.readline()

    success_cmd = "cmd succeeded!"



    # check parameter value
    # update by known value
    # restart (randomly)
    # check again





    timeout_start = time.time()
    timeout = 30  # 30 seconds

    cmd = "param show TEST_1"
    serial_cmd = '{0}; echo "{1}"\n'.format(cmd, success_cmd)
    ser.write(serial_cmd.encode("ascii"))
    ser.flush()
    ser.readline()

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if len(serial_line) > 0:

            print(serial_line, end='')

            if ("] :" in serial_line):
                print(serial_line.split('] :')[1])
                param_value = int(serial_line.split('] :')[1].strip())
                print("param value:", param_value)

                serial_cmd = 'param set {0} {1}\n'.format("TEST_1", param_value + 1)
                print(serial_cmd)
                ser.write(serial_cmd.encode("ascii"))
                ser.flush()

    ser.close()

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=None, help='', required=True)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    do_parameter_restart_test(args.device, args.baudrate)

if __name__ == "__main__":
   main()
