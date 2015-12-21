import threading
import sys
import os
import time
from datetime import datetime, timedelta
import multi_prog

from pynrfjprog import API as nrfjprog
from pynrfjprog import Hex

class JLINKEXE():

    def __init__(self, snr=0, verbose=5, segid=None, info=None ):
        self.snr=snr
        self.verbose=verbose
        self.segid = segid
        self.info = info
        
        self.timeout   = 5
        self.nprog = nrfjprog.API(nrfjprog.DeviceFamily.NRF51)
        self.idle      = threading.Event()
        self.ping_run     = threading.Event()
        self.p_ack     = threading.Event()
        self.started   = threading.Event()
        self.stopped   = threading.Event()
        self.reset_run = threading.Event()
        self.start_run = threading.Event()
        self.stop_run  = threading.Event()
        self.close_run = threading.Event()
        self.listen_run = threading.Event()
        
    def rtt_message(self, msg,rsp):
        self.rtt_init()
        self.rtt_write(msg)
        if (self.verbose > 2):
            sys.stdout.write("sent "+ msg + " to "+str(self.snr)+ "\r\n")
        
        time.sleep(0.5)
        out = ""
        while (True):
            new = self.rtt_read() 
            if new == "":
                break
            else:
                out = out + new
        
        self.rtt_close()
        if rsp in out:
            if (self.verbose > 2):
                sys.stdout.write("snr="+str(self.snr) + " " + " line="+ out + "\r\n")
            return True, out
        else:
            print ("Expected: "+str(msg)+"   but received: "+str(out))
            return False, ""
    

    def rtt_write(self, data):
        return self.nprog.rtt_write(0, data)

    def rtt_init(self):
        global devs
        self.nprog.open()
        if (self.verbose > 2):
            print ("rtt read device="+str(self.segid))
        self.nprog.connect_to_emu_with_snr(int(self.segid))
        self.nprog.rtt_start()
        time.sleep(1)

    def rtt_close(self):
        self.nprog.rtt_stop()
        self.nprog.disconnect_from_emu()
        self.nprog.close()

    def listen(self):
        print ("Starting central RTT and listen")
        self.rtt_init()
        while(self.listen_run.isSet()):
            out = self.rtt_read()
            #read_buf.append(out)
            #print (out)

    def rtt_channels(self):
        down, up = self.nprog.rtt_read_channel_count()
        print("count down="+str(down))
        print("count up="+str(up))

        name,size = self.nprog.rtt_read_channel_info(0, nrfjprog.RTTChannelDirection.UP_DIRECTION)
        print("UP name="+str(name))
        print("UP size="+str(size))
        name,size = self.nprog.rtt_read_channel_info(0, nrfjprog.RTTChannelDirection.DOWN_DIRECTION)
        print("DOWN name="+str(name))
        print("DOWN size="+str(size))

        return True

    def rtt_read(self):
        out = self.nprog.rtt_read(0, 0x100)
        if (self.verbose > 3) and (len(out) !=0):
            sys.stdout.write("rtt_out = " +str(out))
        return out

    def start(self):
        self.start_run.clear()
        rtn = self.rtt_message("S", "START")        
        if rtn:
            self.started.set()
            if (self.verbose > 0):
                sys.stdout.write("snr="+str(self.snr) + " started \r\n")
            if (self.snr == 0):
                TIMER.stage = datetime.now()
                if (self.verbose > 0):
                    sys.stdout.write ("setting Stage timer to NOW"+ "\r\n")
        return rtn
            
    def stop(self):
        self.stop_run.clear()
        if (self.verbose > 2):
            sys.stdout.write("snr="+str(self.snr) + "  send T"+ "\r\n")
        rtn, data = self.rtt_message("T", "STOP")
        
       
        
        if ("data_start" in data) and ("data_end" in data) :
            if (self.verbose > 3):
                sys.stdout.write("snr="+str(self.snr) + "  DATA = "+str(data[:-1])+ "\r\n")  
            
            line= data.split("\r\n")
            line = [x.replace("\x00", "") for x in line]
            a = [i for i,x in enumerate(line) if "data_start" in x]
            #print ("a" + str(a))
            #print (line)
            line = line[a[0]]
            #print (line)
            pack= line.split(",")
            #print (pack)
            a = [i for i,x in enumerate(pack) if x == "data_start"]
            b = [i for i,x in enumerate(pack) if x == "data_end"]
            #print ("a" + str(a))
            #print ("b" + str(b))
            packets = pack[a[0]+1:b[0]]
            #print (packets)
            packets = [int(x,16) for x in packets]
            for dev_from, pkts in enumerate(packets):
                if dev_from < self.info.steps[self.info.num]:
                    if (self.verbose > 1):
                        sys.stdout.write("dev="+str( dev_from) + "  pkt=" + str(pkts) + "\r\n")
                    self.info.data[dev_from] = pkts
                elif (dev_from >= self.info.steps[self.info.num]) and (pkts != 0):
                    if (self.verbose > 1):
                        sys.stdout.write("ERROR: dev="+str( dev_from) + "  should not be receiving any packets\r\n")
                        sys.stdout.write("ERROR: dev="+str( dev_from) + "  pkt=" + str(pkts) + "\r\n")
            
            if (self.verbose > 2):
                sys.stdout.write ("Received all Data"+ "\r\n")
            self.info.stage_done.set()

        self.stopped.set()
        self.started.clear()
        return True

    def portThread(self):
        self.idle.set() 
        while True:
            
            if self.ping_run.isSet():
                assert(self.ping())
                self.ping_run.clear()
                self.idle.set() 

            if self.start_run.isSet():
                assert(self.start())
                self.start_run.clear()

            if self.stop_run.isSet():
                assert(self.stop())
                self.stop_run.clear()

            if self.reset_run.isSet():
                assert(self.reset())
                self.reset_run.clear()

            if self.listen_run.isSet():
                assert(self.listen())
                self.listen_run.clear()

        return
# PYNRFJPROG DLL

    def dll_send_handle(self):
        rtn, data = self.message(("H"+str(self.snr)))        
        if rtn:
            COMMS.handle_up +=1
        return rtn, data
    
    def dll_ping(self):
        self.ping_run.clear()
        rtn, data= self.message("P", "PING")
        if rtn:
            print ("ping received from "+str(self.snr))
            self.p_ack.set()
        return rtn, data

    def dll_reset(self):
        self.reset_run.clear()
        rtn, data = self.send_handle()
        assert(rtn)
        rtn, data = self.ping()
        assert(rtn)
        
        self.idle.set()
        return True
    
     
    def dll_jprog_reset(self):
        self.nprog.open()
        self.nprog.connect_to_emu_with_snr(devices[self.snr])
        self.nprog.sys_reset()
        print ("# Device reset...")
        self.nprog.go()
        print ("# Application running...")
        self.nprog.close()
        return True

    def dll_jprog_program(self,devices, hexfiles):
        print (hexfiles)
        softdevice = Hex.Hex("c:\mesh\sdk81\components\softdevice\s110\hex\s110_softdevice.hex")

        self.nprog.open()
        self.nprog.connect_to_emu_with_snr(devices[i])

        self.nprog.erase_all()
        try:
            program = Hex.Hex(hexfiles[i])
        except Exception as e:
            self.nprog.close()
            print ("# Invalid hexfile provided. Aborting...")
            raise e

        print ("# Writing %s to device %s..." %(str(hexfiles[i]), str(devices[i])))
        for segment in softdevice:
            self.nprog.write(segment.address, segment.data, True)

        for segment in program:
            self.nprog.write(segment.address, segment.data, True)

        self.nprog.sys_reset()
        print ("# Device reset...")
        self.nprog.go()
        print ("# Application running...")
        self.nprog.close()
   
    def dll_program(self):
        global devs
        global hexfiles

        if (self.verbose > 2):
                sys.stdout.write("snr="+str(self.snr) + " programming device="+ str(devices[self.snr])+ "\r\n")
                sys.stdout.write("snr="+str(self.snr) + " with hexfile ="+ str(hexfiles[0]) +"\r\n")

        self.multiprog([self.segid], [hexfiles[0]])
        return True
