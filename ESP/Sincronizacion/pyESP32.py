import time
import serial
import threading


class serialESP32:
    """
    A class for reading and writing data over a serial port, specifically designed for ESP32 communication.

    Attributes:
        port (str): The serial port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
        baudrate (int): The baud rate for communication (default is 9600).
        timeout (float): Timeout for serial read operation (default is 1 second).
        n_vars (int): Expected number of variables in the received data.
        read_delay (float): Delay between reads in the reading thread.
        write_delay (float): Delay between writes in the writing thread.
        serial_port: The serial port object.
        reading_active: Flag indicating whether the reading thread is active.
        read_thread: The thread for reading serial data.
        write_thread: The thread for writing serial data.
        raw_data: A list to store the received data as floats.
    """

    def __init__(self, port, baudrate=9600, n_vars=1):
        """
        Initialize the SerialESP32 instance.

        Args:
            port (str): The serial port to connect to.
            baudrate (int): The baud rate for communication.
            timeout (float): Timeout for serial read operation.
            n_vars (int): Expected number of variables in the received data.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = 1
        self.n_vars = n_vars
        self.read_delay = 0.1  # Delay between reads
        self.write_delay = 0.1  # Delay between writes
        self.serial_port = None
        self.reading_active = False
        self.read_thread = None
        self.write_thread = None
        self.raw_data = [0] * self.n_vars

    def read_serial_data(self):
        """
        Internal method to read data from the serial port continuously.
        """
        while self.reading_active:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    data_list = line.split(',')

                    if len(data_list) == self.n_vars:
                        print("Receiving data.")
                        try:
                            self.raw_data = [float(value) for value in data_list]
                        except ValueError:
                            print(f"Error: Could not convert data to float: {data_list}")
                    else:
                        print(f"Warning: Unexpected number of values received: {len(data_list)}")

            except Exception as e:
                print(f"Error reading serial data: {e}")

            time.sleep(self.read_delay)

    def write_serial_data(self, data=None):
        """
        Internal method to send data to the serial port.

        Args:
            data: The data to be sent.
        """
        while self.reading_active:
            if data is not None:
                try:
                    self.serial_port.write(f"{data}\n".encode('utf-8'))
                    # print(f"Sending: {data}")
                except Exception as e:
                    print(f"Error writing serial data: {e}")
                break  # Stop after sending one message
            time.sleep(self.write_delay)

    def start(self):
        """
        Start the serial communication by opening the port and starting the reading thread.
        """
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Opened serial port {self.port} with baudrate {self.baudrate}")

            self.reading_active = True
            self.read_thread = threading.Thread(target=self.read_serial_data)
            self.read_thread.daemon = True  # Daemon thread will exit when the main program ends
            self.read_thread.start()

        except Exception as e:
            print(f"Error opening serial port: {e}")

    def stop(self):
        """
        Stop the serial communication by closing the port and stopping the threads.
        """
        self.reading_active = False

        if self.serial_port is not None:
            self.serial_port.close()
            print(f"Closed serial port {self.port}")
