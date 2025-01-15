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

def failsafe_swap():
    s.write(b"\xC1")

def finalize_swap():
    s.write(b"\xC2")

def write_record(t, addr, data):
    d = ":{:02x}{:04x}{:02x}".format(len(data), addr, t)
    for i in data:
        d += "{:02x}".format(i)
    cs = sum(int(d[i:i+2],16) for i in range(1, len(d), 2))
    cs = (256 - (cs & 0xff))&0xff
    d += "{:02x}\n".format(cs)
    #print(d, end="")
    s.write(d.encode())
    if t == 0 or t == 7:
        echo = s.readline().decode()
        if echo.upper() != d.upper(): 
            print("ERROR \n      wrote {}\n   received {}".format(d.strip(), echo.strip()))
            return False
    return True


print("Booting {}".format(addr))
boot(addr)


