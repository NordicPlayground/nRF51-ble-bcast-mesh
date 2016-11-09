import sys
import subprocess

PADSIZE = 12
infos = {}
infos["7FFFFFFF"] = ("END", 1)
infos["00010011"] = ("KEY", 17)
infos["00020005"] = ("VERSION", 5)
infos["00040002"] = ("FLAGS", 2)
infos["00100003"] = ("SEGMENT_SD", 3)
infos["00110003"] = ("SEGMENT_BL", 3)
infos["00120003"] = ("SEGMENT_APP", 3)
infos["001A0011"] = ("SIGN_SD", 17)
infos["001B0011"] = ("SIGN_BL", 17)
infos["001C0011"] = ("SIGN_APP", 17)
infos["00210006"] = ("BANK_SD", 6)
infos["00220006"] = ("BANK_BL", 6)
infos["00240006"] = ("BANK_APP", 6)
infos["00210016"] = ("BANK_SD_SIGNED", 21)
infos["00220016"] = ("BANK_BL_SIGNED", 21)
infos["00240016"] = ("BANK_APP_SIGNED", 21)

def nrfjprog(args):
    process = subprocess.Popen("nrfjprog "+args, stdout=subprocess.PIPE, universal_newlines=True)
    out, err = process.communicate()
    if process == None or process.returncode != 0:
        print("Error calling nrfjprog with arguments " + args + ".")
        print(out)
        exit(2)
    return str(out)

is_nrf52 = ("--52" in sys.argv or "--nrf52" in sys.argv)
if is_nrf52:
    family = " -f NRF52"
    device_page_address="0x7f000"
else:
    family = ""
    device_page_address="0x3FC00"

readout = nrfjprog("--memrd "+ device_page_address + " --n 1024" + family)
words = ""
for line in readout.splitlines(True):
    words += line[12:48]
entry = ""
step_count = 0
valid_entry = False
for word in words.split()[1:]:
    if step_count == 0:
        if word.startswith("0000"):
            step_count = int(word, 16) - 1
            valid_entry = False
            continue

        if not word in infos:
            print("\nUnknown entry " + word + ". Device page invalid!")
            break
        (entry, step_count) = infos[word]
        valid_entry = True
        if entry == "END":
            sys.stdout.write("\n" + entry + ".")
            break
        sys.stdout.write("\n" + entry + ": ")
        for i in range(0, PADSIZE - len(entry)):
            sys.stdout.write(" ")
    elif valid_entry:
        sys.stdout.write(word)
    step_count -= 1

