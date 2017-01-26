import binascii
import subprocess
import sys
import serial
import time

def get_device_family(serial_number):
    family = {
        '480': 'NRF51',
        '680': 'NRF51',
        '681': 'NRF51',
        '682': 'NRF52',
    }
    if not serial_number[:3] in family:
        return 'NRF51' # fallback to 51.
    else:
        return family[serial_number[:3]]

def read_serial_event(s):
    evt = ""
    while True:
        length = s.read()
        if ord(length) > 0:
            evt = s.read(ord(length))
            return evt



def print_usage():
    print("")
    print("Usage:\tbootloader_verify.py <Segger serial-number> <COM-port>")

def nrfjprog(args):
    process = subprocess.Popen("nrfjprog "+args, stdout=subprocess.PIPE)
    out, err = process.communicate()
    if process != None and process.returncode == 2:
        print("Couldn't find nrfjprog, exiting.")
        exit(2)
    if process == None or process.returncode != 0:
        print("Error calling nrfjprog with arguments " + args + ".")
        print(out)
        exit(2)
    return out

def read_uicr(serial_number):
    family = get_device_family(serial_number)
    sys.stdout.write("Device family:\t\t\t" + family + "\n")
    sys.stdout.write("Reading UICR..\t\t\t")
    read = nrfjprog("-s " + serial_number + " --memrd 0x10001014 --n 4 --w 32 --family " + family).strip()
    bootloader_addr = str(read).split()[1]
    if bootloader_addr == "FFFFFFFF":
        print("ERROR: UICR NOT SET.")
        print("Checkpoints:")
        print("\tHave you flashed the bootloader with nrfjprog?")
        print("\tDid you flash the Softdevice BEFORE the bootloader?")
        exit(1)

    read = nrfjprog("-s " + serial_number + " --memrd 0x" + bootloader_addr + " --n 4 --w 32 --family " + family).strip()
    bootloader_vector_pointer = str(read).split()[1]
    if bootloader_vector_pointer < "20000000":
        print("ERROR: Bootloader vector pointer invalid.")
        print("Checkpoints:")
        print("\tHave you flashed the bootloader with nrfjprog?")
        print("\tDid you flash the Softdevice BEFORE the bootloader?")
        print("\tDid you erase the device before programming all the hex-files?")
        exit(1)
    if bootloader_vector_pointer == "FFFFFFFF":
        print("ERROR: Bootloader not present.")
        print("Checkpoints:")
        print("\tHave you flashed the bootloader with nrfjprog?")
        print("\tDid you flash the Softdevice BEFORE the bootloader?")
        print("\tDid you erase the device before programming all the hex-files?")
        exit(1)
    print("OK.")
    return bootloader_addr

def read_device_page(serial_number):
    family = get_device_family(serial_number)
    # need to know the flash size to get the device page. Can read this from the FICR:
    ficr = str(nrfjprog("-s " + serial_number + " --memrd 0x10000010 --n 8 --w 32 --family " + family).strip()).split()[1:]
    code_page_size = int(ficr[0], 16)
    code_page_count = int(ficr[1], 16)
    device_page_location = code_page_size * (code_page_count - 1)

    sys.stdout.write("Reading Device page..\t\t")
    device_page = nrfjprog("-s " + serial_number + " --memrd " + hex(device_page_location) + " --n 4 --w 32 --family " + family).strip()
    device_page_header = str(device_page).split()[1]
    if device_page_header == "FFFFFFFF":
        print("ERROR: DEVICE PAGE NOT PRESENT.")
        print("Checkpoints:")
        print("\tHave you flashed the device page?")
        exit(1)
    if device_page_header != "08080104":
        print("ERROR: DEVICE PAGE INVALID.")
        print("Checkpoints:")
        print("\tDid you erase the device before programming all the hex-files?")
        exit(1)
    print("OK.")
    return device_page_header

def reset_device(serial_number, port):
    sys.stdout.write("Resetting device..\t\t")
    try:
        s = serial.Serial(port, 115200, rtscts = True)
    except:
        print("ERROR: Could not open COM port " + port)
        exit(1)
    nrfjprog("-s " + serial_number + " --reset --family " + family)
    time.sleep(0.2)
    response = read_serial_event(s)

    if b"\x81\x02\x00" == response[:3]:
        print("OK (In application)")
    elif not b"\x81\x01\x00" in response:
        print("ERROR: Invalid start sequence from bootloader: " + binascii.hexlify(response))
        print("Checkpoints:")
        print("\tHave you flashed the bootloader with nrfjprog?")
        print("\tDoes your bootloader have serial communication enabled?")
        s.close()
        exit(1)
    else:
        print("OK.")
    time.sleep(0.1)
    s.close()

def echo(port):
    sys.stdout.write("Checking serial connection..\t")
    try:
        s = serial.Serial(port, 115200, rtscts = True)
    except:
        print("ERROR: Could not open COM port " + port)
        exit(1)
    s.write(b"\x03\x02\xaa\xbb")
    time.sleep(0.1)
    if not s.read(4).startswith(b"\x03\x82\xaa\xbb"):
        print("ERROR: Invalid response!")
        print("Checkpoints:")
        s.close()
        exit(1)
    s.close()
    print("OK.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please provide the serial number of your device")
        print_usage()
        exit(1)
    if len(sys.argv) < 3:
        print("Please provide the COM port of your device")
        print_usage()
        exit(1)
    try:
        int(sys.argv[1])
    except:
        print("Invalid serial number " + sys.argv[1])
        print_usage()
        exit(1)

    bootloader_addr = read_uicr(sys.argv[1])
    read_device_page(sys.argv[1])
    reset_device(sys.argv[1], sys.argv[2])
    echo(sys.argv[2])

    print("\nBootloader verification OK.")
