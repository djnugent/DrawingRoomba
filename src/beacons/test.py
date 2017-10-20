import serial
import time
import struct
ser = serial.Serial("COM9",9600)

time.sleep(0.5)

byte1 = 0x42
while(1):
  byte2 = byte1
  byte1 = struct.unpack("B",ser.read(1))[0]
  if byte2 is not None and byte1 == 0x47 and byte2 ==0xff:
      msg = struct.unpack("<H",ser.read(2))[0]
      print("header",hex(msg))
      if msg == 0x11:
          payload_len = struct.unpack("<B",ser.read(1))[0]
          timestamp = struct.unpack("<I",ser.read(4))[0]
          x = struct.unpack("<i",ser.read(4))[0]
          y = struct.unpack("<i",ser.read(4))[0]
          z = struct.unpack("<i",ser.read(4))[0]
          flags = struct.unpack("<B",ser.read(1))[0]
          addr = struct.unpack("<B",ser.read(1))[0]
          reserved = struct.unpack("<I",ser.read(4))[0]
          crc = struct.unpack("<H",ser.read(2))[0]
          print("x: {}mm ,y: {}mm ,z: {}mm".format(x,y,z))
