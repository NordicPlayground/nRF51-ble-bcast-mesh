import time
import logging
import traceback
import threading
import collections
from serial import Serial
from aci import AciEvent, AciCommand

EVT_Q_BUF = 16

class AciDevice(object):
    def __init__(self, device_name):
        self.device_name = device_name
        self._pack_recipients = []
        self._cmd_recipients  = []
        self.lock = threading.Event()
        self.events = list()

    @staticmethod
    def Wait(self, timeout=1):
        if len(self.events) == 0:
            self.lock.wait(timeout)
        self.lock.clear()

        if len(self.events) == 0:
            return None
        else:
            event = self.events[:]
            self.events.clear()
            return event

    def AddPacketRecipient(self, function):
        self._pack_recipients.append(function)

    def AddCommandRecipient(self, function):
        self._cmd_recipients.append(function)

    def ProcessPacket(self, packet):
        self.events.append(packet)
        self.lock.set()
        for fun in self._pack_recipients[:]:
            try:
                fun(packet)
            except:
                logging.error('Exception in pkt handler %r', fun)
                logging.error('traceback: %s', traceback.format_exc())

    def ProcessCommand(self, command):
        for fun in self._cmd_recipients[:]:
            try:
                fun(command)
            except:
                logging.error('Exception in pkt handler %r', fun)
                logging.error('traceback: %s', traceback.format_exc())

    def write_aci_cmd(self, cmd):
        if isinstance(cmd,AciCommand.AciCommandPkt):
            self.WriteData(cmd.serialize())
            retval = self.Wait(self)
            print("Event received: %s" %retval)
            if retval == None:
                logging.info('cmd %s, timeout waiting for event' % (cmd.__class__.__name__))
        else:
            logging.error('The command provided is not valid: %s\nIt must be an instance of the AciCommandPkt class (or one of its subclasses)', str(cmd))



class AciUart(threading.Thread, AciDevice):
    def __init__(self, port, baudrate=115200, device_name=None, rtscts=False):
        self.events_queue = collections.deque(maxlen = EVT_Q_BUF)
        threading.Thread.__init__(self)
        if not device_name:
            device_name = port
        AciDevice.__init__(self, device_name)

        self._write_lock = threading.Lock()

        logging.debug("log Opening port %s, baudrate %s, rtscts %s", port, baudrate, rtscts)
        self.serial = Serial(port=port, baudrate=baudrate, rtscts=rtscts, timeout=0.1)

        self.keep_running = True
        self.start()

    def __del__(self):
        self.stop()

    def stop(self):
        self.keep_running = False

    def get_packet_from_uart(self):
        tmp = bytearray([])
        while self.keep_running:
            tmp += bytearray(self.serial.read())
            tmp_len = len(tmp)
            if tmp_len > 0:
                pkt_len = tmp[0]
                if tmp_len > pkt_len:
                    data = tmp[:pkt_len+1]
                    yield data
                    tmp = tmp[pkt_len+1:]

    def run(self):
        for pkt in self.get_packet_from_uart():
            try:
                pkt = list(pkt)
                if len(pkt) < 2:
                    logging.error('Invalid packet: %r', pkt)
                    continue
                parsedPacket = AciEvent.AciEventDeserialize(pkt)
            except Exception:
                logging.error('Exception with packet %r', pkt)
                logging.error('traceback: %s', traceback.format_exc())
                parsedPacket = None

            if parsedPacket:
                self.events_queue.append(parsedPacket)
                logging.debug('parsedPacket %r %s', parsedPacket, parsedPacket)
                self.ProcessPacket(parsedPacket)

        self.serial.close()
        logging.debug("exited read event")

    def WriteData(self, data):
        with self._write_lock:
            if self.keep_running:
                self.serial.write(bytearray(data))
                self.ProcessCommand(data)

    def __repr__(self):
        return '%s(port="%s", baudrate=%s, device_name="%s")' % (self.__class__.__name__, self.serial.port, self.serial.baudrate, self.device_name)
