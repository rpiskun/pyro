from threading import Thread
import serial
import time
import numpy as np
from numpy_ringbuffer import RingBuffer
import matplotlib.pyplot as plt
import matplotlib.animation as animation

RING_BUF_CAPACITY = 512

ser = serial.Serial('COM3', 115200, timeout=1)

x_timestamp = RingBuffer(capacity = RING_BUF_CAPACITY, dtype = np.integer)
y_inst = RingBuffer(capacity = RING_BUF_CAPACITY, dtype = np.integer)
y_avg = RingBuffer(capacity = RING_BUF_CAPACITY, dtype = np.integer)

offset = 0

def pyro_readdata():
    line = ser.readline()
    if not line:
        return
        
    lst_bytes = list(line)
    if len(lst_bytes) != 10:
        return

    timestamp = int.from_bytes(bytes(lst_bytes[:4]), "big")
    adc_inst = int.from_bytes(bytes(lst_bytes[4:6]), "big", signed = "True")
    adc_avg = int.from_bytes(bytes(lst_bytes[6:8]), "big", signed = "True")
    
    global offset
    if offset == 0:
        offset = timestamp
    
    x_timestamp.append((timestamp - offset))
    y_inst.append(adc_inst)
    y_avg.append(adc_avg)
    
    tt = "[{}: {}; {}]".format(timestamp, adc_inst, adc_avg)
    print(tt)

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.grid(True, linestyle='-')

def animate(i):
    xs_time = np.array(x_timestamp)
    ys_inst = np.array(y_inst)
    ys_avg = np.array(y_avg)
    
    # Draw x and y lists
    ax.clear()
    ax.plot(xs_time, ys_inst, color='#C5C9C7')
    ax.plot(xs_time, ys_avg, color='tab:orange')
    ax.grid(True, linestyle='-')
    ax.tick_params(labelcolor='r', labelsize='medium', width=3)
    
    plt.ylim(-400, 400)
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('PYD1588 data')
    plt.ylabel('Instant/Average Amplitude')
    
    plt.tight_layout()

def pyro_infinite_read():
    while True:
        pyro_readdata()
    
if __name__ == "__main__":
    thread1 = Thread(target = pyro_infinite_read)
    
    thread1.start()
    
    ani = animation.FuncAnimation(fig, animate, interval = 250)
    plt.show()
    
    thread1.join()
    print("thread finished...exiting")
    