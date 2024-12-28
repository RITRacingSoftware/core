import serial
import time
import sys

s = serial.Serial(sys.argv[1], 500000)
addr = int(sys.argv[2])

def boot(addr, timeout=2):
    t = time.time()
    while time.time() - t < timeout:
        s.write(bytearray([0x80 | addr]))
        time.sleep(0.001)

def reset():
    s.write(b"\xC0")

print("Booting {}".format(addr))
boot(addr)
print("\nResetting")
reset()

