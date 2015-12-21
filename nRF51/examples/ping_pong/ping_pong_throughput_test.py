# pep8 --ignore=E221,E501  rtt_throughput.py

from threading import Thread
import threading
import subprocess
import sys
import os
import time
from datetime import datetime, timedelta
import csv
import linecache
import multi_prog
import re
import jlinkexe as COMMS
import argparse

threads = []


class test_info():
    def __init__(self, devs=None, num_tests=None, args=None):
        self.args         = args
        self.num_tests    = args.num_tests
        self.devs         = devs
        self.test_time    = args.test_time
        self.hexfiles     = []
#       for x in args.hex:
        self.hexfiles.append(args.hex)

        if self.args.verbose > 1:
            print("hexfile = " + str(self.hexfiles[0]))
        self.top          = len(self.devs)
        self.steps        = [x for x in range(0, (self.top + 1), (int(self.top / self.num_tests)))]
        self.stage_done   = threading.Event()
        self.view         = [x for x in range(0, self.top, 1)]
        self.data         = [0] * self.top
        self.state_f      = "0x1FC00"
        self.handle_f     = "0x1FC04"
        self.base_f       = "127"
        self.stage_length = timedelta(seconds=self.test_time)
        self.stage_timer  = 0
        self.num          = 0
        self.RBC_MESH_VALUE_MAX_LEN = 23

        try:
            self.steps.remove(1)
        except Exception as exception:
            pass
        sys.stdout.write(str(self.steps) + "\r\n")
        if self.args.verbose > 1:
            print('initialised test_info')


class TEST():
    def __init__(self, info, log, dev):
        self.info = info
        self.log  = log
        self.dev  = dev
        if self.log.verbose > 1:
            print("initialised TEST")

    def start_stage(self, num_devices):
        self.dev.start(0, 1)
        time.sleep(0.1)
        self.dev.start(1, num_devices)
        TIMER.stage = datetime.now()

        return True

    def stop_stage(self, num_devices):
        self.dev.stop(0, 1)
        self.info.stage_timer = TIMER.stage_diff()
        self.info.stage_done.clear()
        self.dev.jlink[0].stop_run.set()
        self.info.stage_done.wait()
        self.log.messages[0:num_devices] = self.info.data[0:num_devices]

        self.dev.erase(0, num_devices)
        self.dev.reset(0, num_devices)
        self.dev.set_handles(0, num_devices)
        return True

    def run(self):
        # Test All devices
        self.log.append_csv()
        self.dev.program()
        self.dev.set_handles(0, self.info.top)
        self.dev.start_threads(0)

        for self.info.num in range(1, len(self.info.steps)):
            if (self.log.verbose > 1):
                sys.stdout.write("Stage [" + str(self.info.num) + "] starting.  [" + str(self.info.steps[self.info.num]) + "] devices\r\n")

            assert(self.start_stage(self.info.steps[self.info.num]))

            TIMER.next_stage = datetime.now() + self.info.stage_length
            while (TIMER.next_stage >= datetime.now()):
                pass

            if (self.log.verbose > 1):
                sys.stdout.write("Stage [" + str(self.info.num) + "] ending.  [" + str(self.info.steps[self.info.num]) + "] devices\r\n")

            assert(self.stop_stage(self.info.steps[self.info.num]))

            if (self.log.verbose > 0):
                self.log.print_perf(self.info.steps[self.info.num], self.info.stage_timer)

            self.log.clr_msgs()

            if self.info.num <= len(self.info.steps) - 1:
                self.log.append_csv()

            if self.info.num == len(self.info.steps) - 1:
                time.sleep(2)
                self.log.print_csv()
                return


class DEVICES():
    def __init__(self, info, log):
        self.info = info
        self.log  = log

        self.snr = 1
        self.ports = []
        self.jlink = []
        if self.log.verbose > 1:
            print("initialised DEVICES")

    def program(self):
        now = TIMER.now()
        success = multi_prog.main(self.info.hexfiles[0], sd=True, flash=True, verbose=self.log.verbose)
        if self.log.verbose > 2:
            print("programming devices took = " + str(TIMER.diff(now, TIMER.now())) + " seconds")
        return success

    def reset(self, start, end):
        now = TIMER.now()
        success = multi_prog.reset_batch(self.info.devs[start:end])
        if self.log.verbose > 2:
            print("resetting devices took = " + str(TIMER.diff(now, TIMER.now())) + " seconds")
        return success

    def set_handles(self, start, end):
        now = TIMER.now()
        vals = []
        for i, device in enumerate(self.info.devs):
            vals.append(str(hex(i)))
        success = multi_prog.memory_batch(self.info.devs[start:end], action="write", addr=self.info.handle_f, val=vals, verbose=self.log.verbose)

        if self.log.verbose > 2:
            print("setting handles took = " + str(TIMER.diff(now, TIMER.now())) + " seconds")
        return success

    def start(self, start, end):
        now = TIMER.now()
        success = multi_prog.memory_batch(self.info.devs[start:end], action="write", addr=self.info.state_f, val="0x0FFF", verbose=self.log.verbose)
        if self.log.verbose > 2:
            print("starting devices took = " + str(TIMER.diff(now, TIMER.now())) + " seconds")
        return success

    def stop(self, start, end):
        now = TIMER.now()
        success = multi_prog.memory_batch(self.info.devs[start:end], action="write", addr=self.info.state_f, val="0x00FF", verbose=self.log.verbose)
        if self.log.verbose > 2:
            print("starting devices took = " + str(TIMER.diff(now, TIMER.now())) + " seconds")
        return success

    def erase(self, start, end):
        now = TIMER.now()
        success = multi_prog.memory_batch(self.info.devs[start:end], action="erasepage", addr=self.info.base_f, verbose=self.log.verbose)
        if self.log.verbose > 2:
            print("erasing devices took = " + str(TIMER.diff(now, TIMER.now())) + " seconds")
        return success

    def print_status(self, num_devices):
        for i in range(0, num_devices):
            if not self.jlink[i].p_ack.isSet():
                sys.stdout.write("port " + str(i) + " is not ping acking" + "\r\n")

    def port_add(self, port):
        self.ports.append(port)

    def start_threads(self, num_devices):
        global threads
        if (self.log.verbose > 1):
            print("start_threads = " + str(self.info.devs[0:num_devices + 1]))
        for idx, port in enumerate(self.info.devs[0:num_devices + 1]):
            if port not in self.ports:
                self.jlink.append(COMMS.JLINKEXE(snr=idx, verbose=self.log.verbose, segid=self.info.devs[idx], info=self.info))
                if (self.log.verbose > 3):
                    sys.stdout.write("starting thread for snr = " + str(idx) + "\r\n")
                thread = Thread(target=self.jlink[idx].portThread, name="SNR=" + str(idx))
                thread.daemon = True
                thread.start()
                threads.append(thread)
                self.port_add(port)

        return True


class LOG():
    def __init__(self, info):
        self.info = info
        self.char = 4
        self.verbose = self.info.args.verbose
        self.throughput  = [0] * self.info.top
        self.messages    = [0] * self.info.top
        self.gateway_log = []
        print("initialised LOG")

    def equal(a, b):
        if (a == b):
            return True
        else:
            return False

    def clr_msgs(self):
        self.messages    = [0] * self.info.top

    def print_perf(self, num_ports, period):
        if (num_ports == 0) or (period == 0.0):
            return
        self.throughput = [(x / (period) * self.info.RBC_MESH_VALUE_MAX_LEN) for x in self.messages]
        sys.stdout.write("\r\n")
        sys.stdout.write("ELAPSED TIME = " + "{0:3.3f}".format(self.info.stage_timer) + " / " + "{0:3.3f}".format(TIMER.clock()))
        sys.stdout.write("-------  GateWay Throughput --------------")
        sys.stdout.write(" STAGE NUMBER= " + "{0:3d}".format(self.info.num) + " ----\r\n")
        sys.stdout.write("             " + ','.join(("{:>" + str(self.char) + "}").format(str(x)) for x in self.info.view) + "\r\n")
        sys.stdout.write("instant msg  " + ','.join(("{:>" + str(self.char) + "}").format(str(x)) for x in [self.messages[i] for i in self.info.view]) + "\r\n")
        sys.stdout.write("instant B/s  " + ','.join(("{0:" + str(self.char) + ".0f}").format(x) for x in [self.throughput[i] for i in self.info.view]) + "\r\n")

    def append_csv(self):
        full_row = []
        full_row.append(self.info.steps[self.info.num])
        for y in range(0, self.info.top):
            full_row.append(("{0:" + str(self.char) + ".0f}").format(self.throughput[y]))

        self.gateway_log.append(full_row)

    def print_csv(self):
        if (self.verbose > 0):
            sys.stdout.write("Printing CSV" + "\r\n")
        sys.stdout.write("Devices, throuput" + "\r\n")
        for x in range(0, len(self.gateway_log)):
            sys.stdout.write(str(self.gateway_log[x]) + "\r\n")

        with open(self.info.args.log, 'a') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=' ')
            spamwriter.writerow(['entry', 'devices', 'throughput'])
            for x in range(0, len(self.gateway_log)):
                full_row = []
                full_row.append(str(x))
                # full_row.append(self.gateway_log[x][0])
                for y in range(0, self.info.top + 1):
                    full_row.append(self.gateway_log[x][y])
                spamwriter.writerow(full_row)
        return True

    def PrintException():
        exc_type, exc_obj, tb = sys.exc_info()
        f = tb.tb_frame
        lineno = tb.tb_lineno
        filename = f.f_code.co_filename
        linecache.checkcache(filename)
        line = linecache.getline(filename, lineno, f.f_globals)
        # sys.stdout.write('EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj))
        sys.stdout.write('EXCEPTION IN (LINE {} "{}"): {}'.format(lineno, line.strip(), exc_obj) + "\r\n")


class TIMER:
    stage = datetime.now()
    start = datetime.now()

    start_stage_period = timedelta(seconds=1)
    next_stage         = datetime.now() + start_stage_period

    def stage_diff():
        return TIMER.diff(TIMER.stage, TIMER.now())

    def clock():
        return TIMER.diff(TIMER.start, TIMER.now())

    def now():
        return datetime.now()

    def diff(start, end):
        return (end.minute - start.minute) * 60 + (end.second - start.second) + (end.microsecond - start.microsecond) / 1000000


def monitorThread(args):
    info = test_info(devs=multi_prog.getDevices(), args=args)
    log  = LOG(info)
    dev  = DEVICES(info, log)
    test = TEST(info, log, dev)

    sys.stdout.write("verbosity = " + str(log.verbose) + "\r\n")

    multi_prog.version()

    test.run()

    os._exit(1)
    return True


def main():
    parser = argparse.ArgumentParser(description='Multi-Link Test')
    parser.add_argument('-x', '--hex', type=str, default="NEED HEX FILE.Try hex")
    parser.add_argument('-l', '--log', type=str, default="NEED LOG FILE.Try log")
    parser.add_argument('-v', '--verbose', type=int, default=0)
    parser.add_argument('-n', '--num_tests', type=int, default=1)
    parser.add_argument('-t', '--test_time', type=int, default=5)
    args = parser.parse_args()

    thread = Thread(target=monitorThread, args=(args,), name="monitor")
    thread.daemon = True
    thread.start()
    threads.append(thread)
    try:
        while True:
            time.sleep(100)
    except (KeyboardInterrupt, SystemExit):
        sys.stdout.write("shutting down" + "\r\n")
        sys.exit(0)

if __name__ == '__main__':
    main()
