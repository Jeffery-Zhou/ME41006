from __future__ import print_function
import time
import serial
print('Ctrl-C to stop')
string=''
sensor = serial.Serial (port='/dev/ttyUSB0',baudrate='9600',timeout=1)    #Open port with baud rate

try:
    
    while True:
        
        data=sensor.read(size=1)
        if data=='\r':
            pass
        elif data=='\n':
            print(string)
            string=''
        else:
            string=string+data
          
except KeyboardInterrupt:
    sensor.close()
    print('Sencor closed!')
