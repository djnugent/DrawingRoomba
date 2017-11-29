from Adafruit_BNO055 import BNO055


bno = BNO055.BNO055(serial_port="/dev/ttyUSB0")

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))
