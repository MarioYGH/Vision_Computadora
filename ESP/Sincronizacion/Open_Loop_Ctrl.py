import time
import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('../utils')

from pyESP32 import *

if __name__ == '__main__':
    # Create a SerialReader instance (adjust port name as needed)
    esp32 = serialESP32(port='/dev/ttyUSB0', baudrate=115200, n_vars=1)

    # Start reading data from the serial port
    esp32.start()

    h = 0.1 # sample time
    tfin = 10 # simulation time
    t = np.arange(0, tfin+h, h)
    N = len(t)

    y = np.zeros(N) #
    u = np.zeros(N) #

    for k in range(N):
        start_time = time.time() # Current time

        # Step input
        if k*h > 3: # step at 3s
            u[k] = 40  # Step at 40%
        else:
            u[k] = 0

        esp32.write_serial_data(u[k])

        y[k] = esp32.raw_data[0]

        elapsed_time = time.time() - start_time # Elapsed time

        time.sleep(h - elapsed_time) # Wait until sampling time is complete

    # Stop the motor
    esp32.write_serial_data(0)

    # Stop the serial reading after simulation time
    esp32.stop()

    with open('input_response.npy', 'wb') as f:
        np.save(f, u)
        np.save(f, y)
        np.save(f, t)
        np.save(f, h)

    plt.figure()
    plt.plot(t, y, label='y(t)')
    plt.plot(t, u, label='u(t)')
    plt.legend(loc='upper left')
    plt.show()
