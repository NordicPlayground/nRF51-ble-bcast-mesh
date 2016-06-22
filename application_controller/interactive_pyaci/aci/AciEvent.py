import logging

MAX_DATA_LENGTH = 30

def AciEventDeserialize(pkt):
    eventLUT = {
        0x81: AciDeviceStarted,
        0x82: AciEchoRsp,
        0x84: AciCmdRsp,
        0xB3: AciEventNew,
        0xB4: AciEventUpdate,
        0xB5: AciEventConflicting,
        0xB6: AciEventTX
    }

    opcode = pkt[1]
    if opcode in eventLUT:
        return eventLUT[opcode](pkt)
    else:
        return AciEventPkt(pkt)

class AciEventPkt(object):
    Len = 1
    OpCode = 0x00
    Data = []
    def __init__(self, pkt):
        self.Len = pkt[0]
        if self.Len == 0 or self.Len > MAX_DATA_LENGTH:
            logging.error("Invalid length: %d, pkt: %s", self.Len, str(pkt) )
        else:
            try:
                self.OpCode = pkt[1]
                if self.Len > 1:
                    self.Data = pkt[2:]
            except:
                logging.error('Packet size must be > 1, packet contents: %s', str(pkt))

    def __repr__(self):
        return str.format("I am %s and my Lenght is %d, OpCode is 0x%02x and Data is %s" %(self.__class__.__name__, self.Len, self.OpCode, self.Data))

class AciDeviceStarted(AciEventPkt):
    #OpCode = 0x81
    def __init__(self,pkt):
        super(AciDeviceStarted, self).__init__(pkt)
        if self.Len != 4:
            logging.error("Invalid length for %s event: %s", self.__class__.__name__, str(pkt))
        else:
            self.OperatingMode = pkt[2]
            self.HWError = pkt[3]
            self.DataCreditAvailable = pkt[4]

class AciEchoRsp(AciEventPkt):
    #OpCode = 0x82
    def __init__(self,pkt):
        super(AciEchoRsp, self).__init__(pkt)

class AciCmdRsp(AciEventPkt):
    #OpCode = 0x84
    def __init__(self,pkt):
        super(AciCmdRsp, self).__init__(pkt)
        if self.Len < 3:
            logging.error("Invalid length for %s event: %s", self.__class__.__name__, str(pkt))
        else:
            self.CommandOpCode = pkt[2]
            self.StatusCode = pkt[3]
            self.Data = self.Data[4:]

class AciEventNew(AciEventPkt):
    #OpCode = 0xB3
    def __init__(self,pkt):
        super(AciEventNew, self).__init__(pkt)
        if self.Len < 3:
            logging.error("Invalid length for %s event: %s", self.__class__.__name__, str(pkt))
        else:
            self.ValueHandle = pkt[2]
            self.Data = self.Data[3:]

class AciEventUpdate(AciEventNew):
    #OpCode = 0xB4
    def __init__(self,pkt):
        super(AciEventUpdate, self).__init__(pkt)

class AciEventConflicting(AciEventNew):
    #OpCode = 0xB5
    def __init__(self,pkt):
        super(AciEventConflicting, self).__init__(pkt)

class AciEventTX(AciEventNew):
    #OpCode = 0xB6
    def __init__(self,pkt):
        super(AciEventTX, self).__init__(pkt)