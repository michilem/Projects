import serial
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

output_file = open('adcLog.csv', 'w')

def animate(i):
    ser.flushInput()
    bytes_ = ser.read()
    print(int.from_bytes(bytes_, "big"))
    byte_as_integer = int.from_bytes(bytes_, "big")
    vout = byte_as_integer * 5.0 / 255
    print(vout)
    output_file.write(f"{byte_as_integer}\n")
    data.append(byte_as_integer)
    ax1.clear()
    ax1.plot(data, 'o')

PORT = "/dev/ttyUSB0"
BAUD = 62500

data = []
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

with serial.Serial(PORT, BAUD) as ser:
    ani = FuncAnimation(fig, animate, interval=0.5, cache_frame_data=False)
    plt.show()

output_file.close()
