import subprocess
from subprocess import call
import argparse
from threading import Thread
import sys
import os
import re
import glob
import time

sdhexfile = "c:\mesh\sdk81\components\softdevice\s110\hex\s110_softdevice.hex"
role= [None]*40
hexfiles = [None]*40

def compile_multi_hex(args, num, custom):
    os.system("rm -rf " + args.build_dir + "/dev*")
    
    for idx in range(num):
        if custom:
            
            os.system("rm -rf " + args.build_dir + "/*.elf")
            f = open( args.src_dir +'/include/config.h', 'w+')
            f.write('#define DEVICE_ADDRESS          0x'+str(args.device_add+idx)+ '\n')
            f.write('#define GROUP_ADDRESS           0x'+args.group_add + '\n')
            f.write('#define DEVICE_ROLE             0x'+str(args.role[idx])      + '\n')
            f.close()
       
        call('ninja -C build')
        hexfiles[idx] = args.build_dir + "/dev" + str(idx) + ".hex"
        cp_command = "cp " + args.hexfile + " " + hexfiles[idx] 
        os.system(cp_command)                
        if custom:
            os.system("cp " + args.src_dir+ '/include/config.h build/config' + str(idx) + ".h" )                
        
        time.sleep(1)

    return 0

def nrfjprog(args):
    process = subprocess.Popen("nrfjprog "+args, stdout=subprocess.PIPE)
    out, err = process.communicate()
    
    if process == None or process.returncode != 0: 
        print( "Error calling nrfjprog with arguments " + args + ".")
        print( out)
        exit(2)
    return out

def getDevices(beginsWith=None):
    devs_b = [dev for dev in nrfjprog("-i").splitlines() if not dev == ""]
    devs = [0] * len(devs_b)
    for i in range(0,len(devs_b)):
       devs[i] = devs_b[i].decode('ascii')
    
    if not beginsWith is None:
        devs = [dev for dev in devs if dev.startswith(beginsWith)]
    
    return devs


def memory_batch(devices, action=None, addr=None, val=None, verbose=None):
    threads = []
    #print (type(val))
    if type(val) == list:
        for i,dev in enumerate(devices):
            thread = Thread(target = memoryDev, args = (dev, action, addr, val[i]))
            thread.start()
            threads.append(thread)
    else:
        for dev in devices:
            thread = Thread(target = memoryDev, args = (dev, action, addr, val))
            thread.start()
            threads.append(thread)

    for thread in threads:
        thread.join()
    if verbose > 2:
        print( "Done.")
    return True

def version():
    out = nrfjprog("-v")
    print(out)
    return out

def memoryDev(device, action=None, addr=None, val=None, verbose=0):
    if device is None:
        snrArg = ""
    else:
        snrArg = "-s " + device + " "
    
    out = ""
    if action == "erasepage":
        out = nrfjprog(snrArg + "--erasepage "+ str(addr))
        if verbose > 2:
            print ("Dev " + str(device) + " erased at:" + str(addr))
    elif action == "write":
        out = nrfjprog(snrArg + "--memwr "    + str(addr) + " --val " + str(val))
        if verbose > 2:
            print ("Dev " + str(device) + "  " + str(val)+ " written at address:" + str(addr))
    elif action == "read":
        out = nrfjprog(snrArg + "--memrd "    + str(addr) + " --n "   + str(val) + " --w 16")
        if verbose > 3:
            print ("Dev " + str(device) + "at address:" + str(addr) + " is \r\n"+ str(out.decode("ascii")))
    
    nrfjprog(snrArg + " --run")
    return out



def progDev(device, args, hex):
    if device is None:
        snrArg = ""
    else:
        snrArg = "-s " + device + " "
    
    nrfjprog(snrArg + "-e")
   
    if args.sd:
        nrfjprog(snrArg + "--program " + sdhexfile)
    
    nrfjprog(snrArg + "--program " + hex)    
    nrfjprog(snrArg + " -p")
    nrfjprog(snrArg + " -r")

def reset_batch(devices, verbose=0):
    threads = []
    for dev in devices:
        thread = Thread(target = resetDev, args = (dev,  ))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
    if verbose > 2:    
        print( "Done.")
 

def resetDev(device):
    if device is None:
        snrArg = ""
    else:
        snrArg = "-s " + device + " "
    
    nrfjprog(snrArg + " -p")
    nrfjprog(snrArg + " -r")

def eraseDev(device):
    nrfjprog("-s " + device + " --eraseall")

def erase_device(devices):
    threads = []
    for dev in devices:
        thread = Thread(target = eraseDev, args = (dev,  ))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
    if verbose > 2:
        print( "Done.")
 

def flash_devices(devices, args, verbose=0):
    threads = []
    idx=0
    for dev in devices:
        if args.multicompile:

            if not dev is None:
                if verbose > 2:
                    print( "Programming device " + dev + " with " + hexfiles[idx])
            thread = Thread(target = progDev, args = (dev, args, hexfiles[idx],  ))
        if args.reset:
            if not dev is None:
                if verbose > 2:
                    print( "Resetting device " + dev )
            thread = Thread(target = resetDev, args = (dev,  ))
        else:
            if not dev is None:
                if verbose > 2:
                    print( "Programming device " + dev + " with " + hexfiles[0])
            thread = Thread(target = progDev, args = (dev, args, hexfiles[0],  ))

        thread.start()
        threads.append(thread)
        idx=idx+1

    for thread in threads:
        thread.join()
    if verbose > 2:
        print( "Done.")

class args:
    src_dir = ""
    snrFilter = ""
    group_add = ""
    role = ""
    sd = ""
    erase = ""
    compil = ""
    multicompile = ""
    return_true = ""
    reset = ""
    flash = ""
    verbose = ""

def main (src_dir, snrFilter=None, group_add=None, role=None, flash=None, \
                       sd=None, erase=None, compil=None, verbose=0, \
                       multicompile=None, return_true=None, reset=None):
    
    #parser = argparse.ArgumentParser(description='Flash mesh')
    #parser.add_argument('-d', '--device_add'       , type=int, default="0290")
    #parser.add_argument('-g', '--group_add'        , type=str, default="f00f")
    #parser.add_argument('--role'             , type=int, nargs='+',default=1)
    #parser.add_argument('--sd'                     , action="store_true")
    #parser.add_argument('-e', '--erase'            , action="store_true")
    #parser.add_argument('-c', '--compile'          , action="store_true")
    #parser.add_argument('-mc', '--multicompile'    , action="store_true")
    #parser.add_argument('-t', '--return_true'      , action="store_true")
    #parser.add_argument('-f', '--flash'            , action="store_true")                       
    #parser.add_argument('-r', '--reset'            , action="store_true")
    #args = parser.parse_args()
    args.src_dir = src_dir
    args.snrFilter = snrFilter
    args.group_add = group_add
    args.role = role
    args.sd = sd
    args.erase = erase
    args.compil = compil
    args.multicompile = multicompile
    args.return_true = return_true
    args.reset = reset
    args.flash = flash
    args.verbose = verbose
    

    
    devices = getDevices(args.snrFilter)
    args.build_dir = "build/"+args.src_dir
    tmp = args.src_dir.split("/")
    args.hexfilename = tmp[-1]
    args.src_dir = args.src_dir.replace("/"+args.hexfilename,"")
    #args.hexfilename = 'rbc_gateway_example.hex'
    #args.hexfilename = 'ping_pong.hex'
    hexfiles[0] = args.src_dir + "/" + args.hexfilename
    
    args.hexfile = args.build_dir + "/" + args.hexfilename + ".hex" 

    print( devices)
    
    if args.erase:
        rtn = erase_device(devices)

    if args.compil:
        rtn = compile_multi_hex(args, len(devices), 0)

    if args.multicompile:
        rtn = compile_multi_hex(args, len(devices), 1)    
    
    if args.flash:
        rtn = flash_devices(devices, args)

    if args.reset:
        rtn = flash_devices(devices, args)
    
    if args.return_true:
        rtn = True

    rtn=0
    return True

if __name__ == '__main__':
    main()