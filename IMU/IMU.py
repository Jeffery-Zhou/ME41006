from __future__ import division
import serial
import codecs
import sys
import os
import math
#start sensor
sensor = serial.Serial(port='/dev/ttyUSB0', baudrate='9600', timeout = 1)
#this function will convert the hex values read from the sensor into integer
def convert(hexVal):
    return int(codecs.encode(hexVal,'hex'), 16)

while True:
    #wait for the starting byte
    data = sensor.read(size = 1)
    if data == b'\x55':
        print('data recieved')
        sensor.read(size = 10)
        break
print('trying',data)

try:
    while True:
        #wait for the starting byte of the position
        data = sensor.read(size = 11)
        if not len(data)==11:
            print("Byte error!")
            break
        if data[1]== b'\x50':
            pass
        #start reading the position
        if data[1]== b'\x53':
            hexVal = []
            for i in range(11):
                hexVal.append(convert(data[i]))
                
            ax = ((hexVal[3]<<8)|(hexVal[2]&0xff)) / 32768 *180
            ay = ((hexVal[5]<<8)|(hexVal[4]&0xff)) / 32768 *180
            az = ((hexVal[7]<<8)|(hexVal[6]&0xff)) / 32768 *180
            
            print(round(ax, 3), round(ay, 3), round(az, 3))
            
except KeyboardInterrupt:
    sensor.close()
    print('Sensor closed!')
