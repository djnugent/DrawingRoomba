import serial
import struct
import time

#notes: big endian
class Roomba:

    def __init__(self,port,baud=115200):
        # Connect
        self.uart = serial.Serial(port,baudrate = baud)
        time.sleep(0.2)
        # Start control
        self.start()
        self.safemode()
        time.sleep(0.5)

    def close(self):
        self.stop()
        self.uart.close()

    def start(self):
        opcode = 128
        cmd = struct.pack("B",opcode)
        self.uart.write(cmd)
        self.uart.flush()

    def stop(self):
        opcode = 173
        cmd = struct.pack("B",opcode)
        self.uart.write(cmd)
        self.uart.flush()
        time.sleep(0.3)

    def safemode(self):
        opcode = 131
        cmd = struct.pack("B",opcode)
        self.uart.write(cmd)
        self.uart.flush()

    def full(self):
        opcode = 132
        cmd = struct.pack("B",opcode)
        self.uart.write(cmd)
        self.uart.flush()

    def reset(self):
        opcode = 7
        cmd = struct.pack("B",opcode)
        self.uart.write(cmd)
        self.uart.flush()


    # velocity ~ mm/s ~ (-500 – 500 mm/s)
    # radius ~ mm ~ (-2000 – 2000 mm)
    def drive(self,velocity,radius):
        opcode = 137
        if not -500 <= velocity <= 500:
            raise ValueError("Velocity out of range: -500 - 500 mm/s")
        if not -500 <= velocity <= 500:
            raise ValueError("Radius out of range: -2000 - 2000 mm")
        cmd = struct.pack(">Bhh",opcode,int(velocity),int(radius))
        print(cmd)
        self.uart.write(cmd)
        self.uart.flush()

    def encoders_



if __name__=="__main__":
    import time
    print("Connect")
    robot = Roomba("COM5")
    try:
        print("Drive")
        robot.drive(100,0)
        time.sleep(5)
        print("Stop")
        robot.drive(0,0)
    finally:
        robot.close()
