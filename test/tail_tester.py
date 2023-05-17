from functools import reduce
import numpy as np
import operator
from serial import Serial, SerialException
import time


class TailController:
    def __init__(self, port, baudrate, sleeping_time=0.1):
        self.serial = Serial()
        self.port = port
        self.baudrate = baudrate
        self.buffer = ""
        self.sleeping_time = sleeping_time

    def start(self):
        # Configure the serial
        self.serial.port = self.port
        self.serial.baudrate = self.baudrate

        # Try to open the serial
        try:
            self.serial.open()
        except SerialException as se:
            print(f"{se.strerror}")
            return

        print("Serial opened")

        while self.serial.is_open:
            # Reading incoming bytes into the buffer
            try:
                self.buffer += (self.serial.read_all()).decode()
            except:
                pass

            # If a complete frame is in the buffer
            if "\n" in self.buffer:
                frame, self.buffer = self.buffer.split("\r\n", 1)
                # Print the frame
                print(frame)

            # Write RHACT frame
            frame = self.RHACT()
            self.serial.write(frame.encode())
            print(frame[:-2])
            time.sleep(self.sleeping_time)

    def RHACT(self):
        ''' Generate RHACT nmea frames to orient D Fin '''
        value = int(500 * np.sin(time.time() / np.pi) + 1500)
        rhact = f"RHACT,{value},1500,1500,1500"
        checksum = reduce(operator.xor, (ord(s) for s in rhact), 0)
        return "$" + rhact + f"*{checksum:02X}\r\n"


if __name__ == "__main__":
    baudrate = 115200
    port = "/dev/ttyUSB0"

    # Opening serial
    tc = TailController(port, baudrate)
    tc.start()
