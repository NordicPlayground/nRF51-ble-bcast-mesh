import logging

def AciCommandLookUp(CommandOpCode):
    commandNameLUT = {
        AciEcho.OpCode: "Echo",
        AciRadioReset.OpCode: "RadioReset",
        AciInit.OpCode: "Init",
        AciValueSet.OpCode: "ValueSet",
        AciValueEnable.OpCode: "ValueEnable",
        AciValueDisable.OpCode: "ValueDisable",
        AciStart.OpCode: "Start",
        AciStop.OpCode: "Stop",
        AciFlagSet.OpCode: "FlagSet",
        AciFlagGet.OpCode: "FlagGet",
        AciDfuData.OpCode: "DfuData",
        AciValueGet.OpCode: "ValueGet",
        AciBuildVersionGet.OpCode: "BuildVersionGet",
        AciAccessAddressGet.OpCode: "AccessAddressGet",
        AciChannelGet.OpCode: "ChannelGet",
        AciIntervalMinMsGet.OpCode: "IntervalMinMsGet",
    }

    if CommandOpCode in commandNameLUT:
        return commandNameLUT[CommandOpCode]
    else:
        return "UNKNOWN COMMAND: 0x%02x" %CommandOpCode

def valueToByteArray(val, no_bytes):
    data = [(val>>(x*8))&0xFF for x in range(no_bytes)]
    return data

class AciCommandPkt(object):
    Data = list()
    Len = 1
    Data = []
    def __init__(self, OpCode, data=[], length = 1):
        self.Len = length
        self.OpCode = OpCode
        self.Data = data
        if length != (len(self.Data) + 1):
            logging.error("Length: %d, must be equal to the number of data items: %s plus 1 (for the opcode)",self.Len, str(self.Data))

    def serialize(self):
        pkt = [self.Len]
        pkt.append(self.OpCode)
        pkt.extend(self.Data)
        return pkt

    def __repr__(self):
        return str.format("I am %s and my Lenght is %d, OpCode is 0x%02x and Data is %s" %(self.__class__.__name__, self.Len, self.OpCode, self.Data))

class AciEcho(AciCommandPkt):
    OpCode = 0x02
    MAX_ECHO_LENGTH = 30
    def __init__(self, data=[], length=1):
        if length > self.MAX_ECHO_LENGTH:
            logging.error("ECHO command can have a maximum of %d byte packet size (including the opcode), not %d",MAX_ECHO_LENGTH,length)
        else:
            super(AciEcho, self).__init__(length=length,OpCode=self.OpCode, data = data)

class AciRadioReset(AciCommandPkt):
    OpCode = 0x0E
    Length = 1
    def __init__(self):
        super(AciRadioReset, self).__init__(length=self.Length,OpCode=self.OpCode)

class AciInit(AciCommandPkt):
    OpCode = 0x70
    Length = 10
    def __init__(self, access_address, min_interval, channel):
        payload = valueToByteArray(access_address,4)
        payload.extend(valueToByteArray(min_interval,4))
        payload.extend(valueToByteArray(channel,1))
        super(AciInit, self).__init__(length=self.Length,OpCode=self.OpCode, data=payload)

class AciValueSet(AciCommandPkt):
    OpCode = 0x71
    MAX_VAL_LENGTH = 26
    def __init__(self, handle, data, length=3):
        if length > self.MAX_VAL_LENGTH:
            logging.error("VALUE_SET command can have a maximum of %d byte packet size (including the opcode), not %d",MAX_VAL_LENGTH,length)
        else:
            payload = valueToByteArray(handle,2)
            payload.extend(data)
            super(AciValueSet, self).__init__(length=length, OpCode=self.OpCode, data=payload)

class AciValueEnable(AciCommandPkt):
    OpCode = 0x72
    Length = 3
    def __init__(self, handle):
        payload = valueToByteArray(handle,2)
        super(AciValueEnable, self).__init__(length=self.Length, OpCode=self.OpCode, data=payload)

class AciValueDisable(AciCommandPkt):
    OpCode = 0x73
    Length = 3
    def __init__(self, handle):
        payload = valueToByteArray(handle,2)
        super(AciValueDisable, self).__init__(length=self.Length, OpCode=self.OpCode, data=payload)

class AciStart(AciCommandPkt):
    OpCode = 0x74
    Length = 1
    def __init__(self):
        super(AciStart, self).__init__(length=self.Length,OpCode=self.OpCode)

class AciStop(AciCommandPkt):
    OpCode = 0x75
    Length = 1
    def __init__(self):
        super(AciStop, self).__init__(length=self.Length,OpCode=self.OpCode)

class AciFlagSet(AciCommandPkt):
    OpCode = 0x76
    Length = 5
    def __init__(self, handle, flag_index, flag_value):
        payload = valueToByteArray(handle,2)
        payload.extend(valueToByteArray(flag_index,1))
        payload.extend(valueToByteArray(flag_value,1))
        super(AciFlagSet, self).__init__(length=self.Length,OpCode=self.OpCode, data=payload)

class AciFlagGet(AciCommandPkt):
    OpCode = 0x77
    Length = 4
    def __init__(self, handle, flag_index):
        payload = valueToByteArray(handle,2)
        payload.extend(valueToByteArray(flag_index,1))
        super(AciFlagGet, self).__init__(length=self.Length,OpCode=self.OpCode, data=payload)

class AciDfuData(AciCommandPkt):
    OpCode = 0x78
    MAX_DFU_LENGTH = 26
    def __init__(self, data=[], length=1):
        if length > self.MAX_DFU_LENGTH:
            logging.error("DFU_DATA command can have a maximum of %d byte packet size (including the opcode), not %d",MAX_DFU_LENGTH,length)
        else:
            super(AciDfuData, self).__init__(length=length,OpCode=self.OpCode, data = data)

class AciValueGet(AciCommandPkt):
    OpCode = 0x7A
    Length = 3
    def __init__(self, handle):
        payload = valueToByteArray(handle,2)
        super(AciValueGet, self).__init__(length=self.Length, OpCode=self.OpCode, data=payload)

class AciBuildVersionGet(AciCommandPkt):
    OpCode = 0x7B
    Length = 1
    def __init__(self):
        super(AciBuildVersionGet, self).__init__(length=self.Length,OpCode=self.OpCode)

class AciAccessAddressGet(AciCommandPkt):
    OpCode = 0x7C
    Length = 1
    def __init__(self):
        super(AciAccessAddressGet, self).__init__(length=self.Length,OpCode=self.OpCode)

class AciChannelGet(AciCommandPkt):
    OpCode = 0x7D
    Length = 1
    def __init__(self):
        super(AciChannelGet, self).__init__(length=self.Length,OpCode=self.OpCode)

class AciIntervalMinMsGet(AciCommandPkt):
    OpCode = 0x7F
    Length = 1
    def __init__(self):
        super(AciIntervalMinMsGet, self).__init__(length=self.Length,OpCode=self.OpCode)
