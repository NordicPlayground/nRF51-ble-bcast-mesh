from threading import Thread
import subprocess
import sys
import os
import time
import datetime
import serial
import serial.tools.list_ports

SEGGER_VID = 1366
BAUDRATE = 460800;
verbose = False
flow_control = True
startTime = datetime.datetime.now()
central = None
snr_max = 0

def printUsage():
    print "Usage: ping_pong.py [<baudrate>] [-f] [-v] [-c <port>]"
    print "\t<baudrate>\tThe desired baudrate. If no argument is given, the baudrate defaults to " + str(BAUDRATE) + "."
    print "\t-f\t\tEnable RTS/CTS flow control"
    print "\t-v\t\tEnable all event logging from central device"
    print "\t-c <port>\t\tForce the designated com port to act as central"

if "-h" in sys.argv or "--help" in sys.argv:
    printUsage()
    exit(0)

if "-f" in sys.argv:
    flow_control = True

if "-v" in sys.argv:
    verbose = True
if "-c" in sys.argv:
    central_index = sys.argv.index("-c") + 1
    if len(sys.argv) <= central_index:
        printUsage()
        exit(160) #bad arguments
    central = sys.argv[central_index]


for arg in sys.argv:
    try:
        BAUDRATE = int(sys.argv[1])
        if BAUDRATE is None:
            printUsage()
            exit(160)
    except:
        pass


def getPorts():
    if sys.platform is "Windows":
        ports = [(name, hwid[12:16]) for (name, desc, hwid) in serial.tools.list_ports.comports()]
        portnames = [name for (name, vid) in ports if vid == str(SEGGER_VID)]
    else:
        portnames = [port[0] for port in serial.tools.list_ports.comports()]
    return portnames

def portThread(port, snr):
    global startTime
    global snr_max
    s = None
    try:
        s = serial.Serial(port, BAUDRATE, rtscts = flow_control)
    except:
        if not s is None:
            s.close()
        sys.stdout.write("Failed to establish connection to " + port + " (handle " + str(snr) + ")\n")
        return
    msgnum = 1
    try:
        s.write(str(snr) + "\r\n")
        sys.stdout.write("Assigned handle " + str(snr) + " to " + port + "\r\n")
        prevtime = datetime.datetime.now()
        while True:
            data = s.read()
            if snr is 0:
                sys.stdout.write(data)

    except Exception, e:
        print e
        sys.stdout.write("Lost " + port + "\n")
        return

threads = []

def monitorThread():
    global central
    global snr_max
    ports = []
    snr = 1
    while True:
        current_ports = getPorts()
        for port in current_ports:
            if not port in ports:
                this_snr = snr
                snr_max = int(snr)
                if central == None or central == port: #force central
                    this_snr = 0
                    central = port
                else:
                    snr += 1
                thread = Thread(target = portThread, args = (port, this_snr), name = port)
                thread.daemon = True
                thread.start()
                threads.append(thread)
        ports = current_ports
        time.sleep(1)


thread = Thread(target = monitorThread, name = "monitor")
thread.daemon = True
thread.start()
threads.append(thread)
try:
    while True: time.sleep(100)
except (KeyboardInterrupt, SystemExit):
    sys.exit(0)

