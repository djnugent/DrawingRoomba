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

        # State variables
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        # Zero out encoders
        self.last_left_ticks, self.last_right_ticks = self._raw_encoders()


    # Send command over serial
    def _send(self,fmt,*data):
        cmd = struct.pack(fmt,*data)
        self.uart.write(cmd)
        self.uart.flush()

    # Close the connection with Roomba
    def close(self):
        self.stop()
        time.sleep(0.3)
        self.uart.close()

    # Instruct Open interface to start
    def start(self):
        opcode = 128
        self._send("B",opcode)

    # Instruct Open interface to stop
    def stop(self):
        opcode = 173
        self._send("B",opcode)

    # Enable remote operation with safety sensors
    def safemode(self):
        opcode = 131
        self._send("B",opcode)

    # Enable remote operation with no safety sensors
    def fullmode(self):
        opcode = 132
        self._send("B",opcode)

    # Reset the Roomba
    def reset(self):
        opcode = 7
        self._send("B",opcode)

    # Command the roomba to move
    # velocity ~ m/s ~ (-0.5 – 0.5 m/s)
    # radius ~ m ~ (-2.0 – 2.0 m)
    def drive(self,velocity,radius):
        opcode = 137

        if not -0.5 <= velocity <= 0.5:
            raise ValueError("Velocity out of range: -0.5 - 0.5 m/s")
        if not -2.0 <= radius <= 2.0:
            raise ValueError("Radius out of range: -2.0 - 2.0 m")

        self._send(">Bhh",opcode,int(velocity*1000),int(radius*1000))

    # Get change in encoder in ticks
    # TODO test overflow and underflow
    def _raw_encoders(self):
        opcode = 142
        left_encoder = 43
        right_encoder = 44
        # Request left encoder ticks
        self._send("BB",opcode,left_encoder)
        # Request right encoder ticks
        self._send("BB",opcode,right_encoder)
        # read values
        cumm_left_ticks = struct.unpack(">H",self.uart.read(2))[0]
        cumm_right_ticks = struct.unpack(">H",self.uart.read(2))[0]

        # Check for left overflow
        diff_ticks_left = cumm_left_ticks - self.last_left_ticks
        if diff_ticks_left < -60000: #reverse overflow
            diff_ticks_left -= 65536
        elif diff_ticks_left > 60000: #foward overflow
            diff_ticks_left = 65536 - diff_ticks_left
        self.last_left_ticks = cumm_left_ticks
        # Check for right overflow
        diff_ticks_right = cumm_right_ticks - self.last_right_ticks
        if diff_ticks_right < -60000: #reverse overflow
            diff_ticks_right -= 65536
        elif diff_ticks_right > 60000: #foward overflow
            diff_ticks_right = 65536 - diff_ticks_right
        self.last_right_ticks = cumm_right_ticks

        return diff_ticks_left, diff_ticks_right


    # Get change in encoder in meters
    # TODO test overflow and underflow
    def encoders(self):
        diff_ticks_left, diff_ticks_right = self._raw_encoders()

        # convert to meters
        mtrs_per_tick = math.pi * 0.072 / 508.8 # 0.072 m diameter wheel and 508.8 ticks per revolution
        left = diff_ticks_left * mtrs_per_tick
        right = diff_ticks_right * mtrs_per_tick
        return  left, right

    # returns distance traveled since last request in meters
    def distance(self):
        opcode = 142
        # send distance request
        self._send("BB",opcode,19)
        # read result
        dist = struct.unpack(">h",self.uart.read(2))[0]
        return dist/1000.0

    # returns angles turned since last request in radians
    def angle(self):
        opcode = 142
        # send angle request
        self._send("BB",opcode,20)
        # read result
        angle = struct.unpack(">h",self.uart.read(2))[0]
        return math.radians(angle)




if __name__=="__main__":
    import time
    print("Connect")
    robot = Roomba("COM10")
    try:
        print("Drive")
        robot.drive(0.1,0)
        start = time.time()
        for i in range(0,120):
            robot.encoders()
        print("hz", 120.0/(time.time()-start))
        print(robot.encoders())
        print("distance",robot.distance())
        print("Stop")
        robot.drive(0,0)
    finally:
        robot.close()
