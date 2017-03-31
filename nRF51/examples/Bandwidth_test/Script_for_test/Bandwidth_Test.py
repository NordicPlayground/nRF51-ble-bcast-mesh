from __future__ import division
from __future__ import print_function

from builtins import int

from pynrfjprog import MultiAPI, Hex
import binascii
import os
import sys 
###########################

master_segger_id =0
softdevice_file_location=[] 
master_hex_file_location=[]
slave_hex_file_location=[] 
flag_ack=[]

############################s

device_array =[]
handle_array=[]
device_array_for_slaves=[]
rtt_buffer_size = 1024

###########################################

def device_discovery():
  print('Entering device_discovery()')
 
  api = MultiAPI.MultiAPI('NRF51')
  api.open()
  
  global device_array 
  global master_segger_id
  
  device_array = api.enum_emu_snr()
  device_array.remove(master_segger_id)
  device_array.insert(0,master_segger_id)
  
  print(device_array)

  for device_serial in device_array:
      
      if (device_serial == None):
          break
      if (device_serial == master_segger_id):
          print ('Master is found ',master_segger_id )
      
  api.close() 
###################################################



    
##########################################################################

def hex_load():
  print('Entering hex_load()')
  api = MultiAPI.MultiAPI('NRF51')
  
  global device_array
  global handle_array 
  global master_segger_id
  global softdevice_file_location 
  global master_hex_file_location 
  global slave_hex_file_location 
  
  a = 0
  #handle number 
  name =1 

  for device_serial in device_array:
    
    if (device_serial!= master_segger_id): 
  
      print (device_serial)

      if (device_serial == None):
          break
		  
      api.open()
          
      api.connect_to_emu_with_snr(device_serial)
      print('Connected to',device_serial,' device.') 
 
      api.erase_all() 

      # Program device with all hex files supplied by user.
      hexFiles = [softdevice_file_location ,slave_hex_file_location]
	  
      for hex_file_path in hexFiles: 

          if (hex_file_path == None): 
               break
          try:
               program = Hex.Hex(hex_file_path) # Parse .hex file into segments. Checks whether user passed a valid file path or not. 
          except Exception as e: # If hex_file_path not a valid file, print an error and raise an exception.
               api.close()
               print(' # One of the hex files provided was invalid... Please check that the path is correct. Closing api and exiting. ')
               raise e

          print('# Writing %s to device  ' % hex_file_path)
          for segment in program: # Program hex file to the device.
               api.write(segment.address, segment.data, True)
     
      #write handle for each slave   
      api.write_u32(0x3F000,name,True)
      handle_array.append(name)

      print ('handle is written')

      # Reset device, run
      api.sys_reset()                         # Reset device
      api.go()                                # Run application
      print('# Application running on',device_serial)
      print (api.read_u32(0x3F000))
      # Close API
      api.close()                             # Close the dll
      a=1
      name = name+1
      print('# Hex file downloaded done on',device_serial)

    if (device_serial == master_segger_id): 
       api.open()
       api.connect_to_emu_with_snr(device_serial)
       print('Connected to',device_serial,' device.') 
 
       api.erase_all()
       hexFiles = [softdevice_file_location ,master_hex_file_location]
	   
       for hex_file_path in hexFiles: 

          if (hex_file_path == None): 
               break
          try:
               program = Hex.Hex(hex_file_path) # Parse .hex file into segments. Checks whether user passed a valid file path or not. 
          except Exception as e: # If hex_file_path not a valid file, print an error and raise an exception.
               api.close()
               print(' # One of the hex files provided was invalid... Please check that the path is correct. Closing api and exiting. ')
               raise e

          print('# Writing %s to device  ' % hex_file_path)
          for segment in program: # Program hex file to the device.
               api.write(segment.address, segment.data, True)
       # Reset device, run
       api.sys_reset()                         # Reset device
       api.go()                                # Run application
       api.close()                             # Close the dll
       print('# Hex file downloaded done on master',device_serial)
##########################################################################


def data_log():
  print('Entering data_log()')
  
  global device_array
  global handle_array
  
  count=len (device_array)
  
  f_array=[]
  f_array.append(open("handle_0_rbc_mesh_example.log", "w"))
  
  i=0
  while (i<len(handle_array)):
     f_array.append(open("handle_%d_rbc_mesh_example.log" %(handle_array[i]), "w"))
     i=i+1

  f = open("_rbc_mesh_example.log", "w") 
  f_master_summary = open("_rbc_mesh_example_master_summary.log", "w")

 
 # print ('count', count)
  print('File is open for logging')

  api_array=[]
  for device_serial in device_array :
    if (device_serial == None):
          break
    api_array.append (MultiAPI.MultiAPI('NRF51'))

  i=0
  while (i<count): 
    api_array[i].open()
    api_array[i].connect_to_emu_with_snr(device_array[i],8000)
    api_array[i].sys_reset()
    api_array[i].rtt_start()
    print('Connected to',device_array[i],' device.')
    i=i+1

  import datetime
  import time
  ts0 = time.time()
  print ('ts0',datetime.datetime.fromtimestamp(ts0).strftime('%Y-%m-%d %H:%M:%S'))

  i=0
  while (i<count):  
      api_array[i].go() 
      i=i+1

  i=0
  while (i<count):
       while not api_array[i].rtt_is_control_block_found():
          # print ('RTT control block has not been found')
          pass
       i=i+1

  
  global rtt_buffer_size
  global simulation_time
 
  ts1 = time.time()  
  st1 = datetime.datetime.fromtimestamp(ts1).strftime('%Y-%m-%d %H:%M:%S')
  print ('Simulation started at',st1)

  while True:
    
    i=0
    while (i<count): 
         buffer_val = api_array[i].rtt_read(0, rtt_buffer_size)
         if buffer_val:
            f_array[i].write('\n')
            f_array[i].write(str(device_array[i]))
            f_array[i].write(buffer_val)
            f.write('\n')
            f.write(str(device_array[i]))
            f.write(buffer_val)
         i=i+1

    ts2= time.time()

    if ((ts2-ts0)> simulation_time):
       break


  total_run_time = ts2 -ts0

  st2 = datetime.datetime.fromtimestamp(ts2).strftime('%Y-%m-%d %H:%M:%S')
  print ('Simulation ends at',st2)
 


########################### Calculate Received packet in Master from all slaves ####################################################  

  byte_fetch= count* 4 
  
  packet_array_from_memory = api_array[0].read(0x20004688,byte_fetch)  
  packet_array=[]
  

  i=7
  while (i<= byte_fetch):
     pkt=(packet_array_from_memory[i]*(2**24))+(packet_array_from_memory[i-1]*(2**16))+(packet_array_from_memory[i-2]*(2**8))+(packet_array_from_memory[i-3])
     packet_array.append(pkt)
     i=i+4
	 
  total_received_packet = 0
  for i in packet_array:
      if (i== None):
         break
      total_received_packet = total_received_packet +i
###############################################################################  



########################## Writing to summary file ###############################################
  global flag_ack
  i=1
  while (i<count):
     f_master_summary.write("%d handle %d total pkt rcv %d\n" % (device_array[i],handle_array[i-1],packet_array[handle_array[i-1]-1]))
     i=i+1

  if (flag_ack== 'without_ack'):	 
     f_master_summary.write("Total pkt received %d run time %ds Bandwidth %d bps \n" % (total_received_packet,total_run_time,((total_received_packet*23*8)/total_run_time)))
  if (flag_ack== 'with_ack'):	 
     f_master_summary.write("Total pkt received %d run time %ds Bandwidth %d bps \n" % (total_received_packet,total_run_time,((total_received_packet*23*8 + total_received_packet*8 )/total_run_time)))
              
#################################################################################################  
         
  i=0
  while (i<count):  
      api_array[i].close() 
      f_array[i].close()
      i=i+1
  f_master_summary.close()
  if (flag_ack== 'without_ack'):
	print ('Total received packet',total_received_packet,'Bandwidth',((total_received_packet*23*8)/total_run_time),'bps')
  if (flag_ack== 'with_ack'):
	print ('Total received packet',total_received_packet,'Bandwidth',((total_received_packet*23*8 + total_received_packet*8 )/total_run_time),'bps')
   
def main(argv):

   	global master_segger_id 
    	global softdevice_file_location
    	global master_hex_file_location
    	global slave_hex_file_location 
    	global simulation_time
	global flag_ack
	
        master_segger_id = int(argv[1])	
	softdevice_file_location= argv[2] 
	master_hex_file_location= argv[3] 
	slave_hex_file_location = argv[4]
	simulation_time=int(argv[5])
	flag_ack=argv[6]
	
	device_discovery() #get all the device connected with the PC    
	hex_load()
	data_log() 
   
if __name__ == '__main__':  
	
	main(sys.argv[0:])
      

     













