import time
import logging
import traceback
import threading
import collections
from serial import Serial
from aci import AciEvent

EVT_Q_BUF = 16

class AciDevice(object):
    def __init__(self, device_name):
        self.device_name = device_name
        self.log = logging.getLogger(((8 - len(device_name)) * ' ' + device_name))
        self._pack_recipients = []
        self._cmd_recipients  = []

    def ProcessPacket(self, packet):
        for fun in self._pack_recipients[:]:
            try:
                fun(packet)
            except:
                self.log.error('Exception in pkt handler %r', fun)
                self.log.error('traceback: %s', traceback.format_exc())

    def ProcessCommand(self, command):
        for fun in self._cmd_recipients[:]:
            try:
                fun(command)
            except:
                self.log.error('Exception in pkt handler %r', fun)
                self.log.error('traceback: %s', traceback.format_exc())

class ACIUart(threading.Thread, AciDevice):
    events_queue = collections.deque(maxlen = EVT_Q_BUF)
    def __init__(self, port, baudrate=115200, device_name=None, rtscts=False):
        threading.Thread.__init__(self)
        if not device_name:
            device_name = port
        AciDevice.__init__(self, device_name)

        self._write_lock = threading.Lock()

        self.log.debug("log Opening port %s, baudrate %s, rtscts %s", port, baudrate, rtscts)
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
                    #btsnoop.write_hci_log(False, event, data)
                    yield data
                    tmp = tmp[pkt_len+1:]

    def run(self):
        for pkt in self.get_packet_from_uart():
            try:
                pkt = list(pkt)
                parsedPacket = AciEvent.AciEventDeserialize(pkt)
                events_queue.append(parsedPacket)
            except Exception:
                self.log.error('Exception with packet %r', pkt)
                self.log.error('traceback: %s', traceback.format_exc())
                parsedPacket = None

            if parsedPacket:
                self.log.debug('parsedPacket %r %s', parsedPacket, parsedPacket)
                #btsnoop.write_hci_log(True, pkt, parsedPacket)
                self.ProcessPacket(parsedPacket)

        self.serial.close()
        self.log.debug("exited read event")

    def WriteData(self, data):
        with self._write_lock:
            if self.keep_running:
                #self.log.info('uart tx -->: %r' % bytearray(cmd))
                #btsnoop.write_hci_log(False, cmd, data)
                self.serial.write(bytearray(data))
                #self.ProcessCommand(data)

    def __repr__(self):
        return '%s(port="%s", baudrate=%s, device_name="%s")' % (self.__class__.__name__, self.serial.port, self.serial.baudrate, self.device_name)
