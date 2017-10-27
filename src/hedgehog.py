import serial
import time
import struct


class Hedgehog:
    def __init__(self,port):
        self.ser = serial.Serial(port,9600)
        self.lock = threading.Lock()
        self.pos = -1,-1,-1 # meters
        self.unread = False

        # start a background thread
        self.t = threading.Thread(target=self.main)
        self.t.daemon = True
        self.running = True
        self.t.start()

    def main(self):
        time.sleep(0.2)

        byte2 = 0x42
        while self.running:
            byte1 = byte2
            byte2 = struct.unpack("B",self.ser.read(1))[0]
            # Align packet using 2 byte header 0xff47
            if byte1 == 0xff and byte2 == 0x47:
              # read packet ID
              msg = struct.unpack("<H",self.ser.read(2))[0]
              if msg == 0x11:
                  # read packet
                  payload_len = struct.unpack("<B",self.ser.read(1))[0]
                  timestamp = struct.unpack("<I",self.ser.read(4))[0]
                  x = struct.unpack("<i",self.ser.read(4))[0]
                  y = struct.unpack("<i",self.ser.read(4))[0]
                  z = struct.unpack("<i",self.ser.read(4))[0]
                  flags = struct.unpack("<B",self.ser.read(1))[0]
                  addr = struct.unpack("<B",self.ser.read(1))[0]
                  reserved = struct.unpack("<I",self.ser.read(4))[0]
                  crc = struct.unpack("<H",self.ser.read(2))[0]
                  # read flag bit mask
                  valid = (flags & 0b00000001) > 0

                  if valid:
                      self.lock.acquire()
                      self.pos = x /1000.0, y /1000.0, z /1000.0
                      self.unread = True
                      self.lock.release()

    def read(self):
        self.lock.acquire()
        x,y,z = self.pos()
        unread = self.unread
        self.unread = False
        self.lock.release()

        return x,y,z,unread


    def close(self):
        self.lock.acquire()
        self.running = False
        self.lock.release()
        self.t.join()


if __name__=="__main__":
    rate = 20 # hz
    hh = Hedgehog("COM9")
    try:
        while True:
            print(hh.read())
            time.sleep(1.0/rate)
    finally:
        hh.close()
