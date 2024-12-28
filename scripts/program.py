import serial
import time
import sys

s = serial.Serial(sys.argv[1], 500000)
addr = int(sys.argv[2])
fname = sys.argv[3]

def boot(addr, timeout=2):
    t = time.time()
    while time.time() - t < timeout:
        s.write(bytearray([0x80 | addr]))
        time.sleep(0.001)

def reset():
    s.write(b"\xC0")

print("Booting {}".format(addr))
boot(addr)
print("Writing...")
dwords = {}
buf = bytearray([])

def write_record(t, addr, data):
    d = ":{:02x}{:04x}{:02x}".format(len(data), addr, t)
    for i in data:
        d += "{:02x}".format(i)
    cs = sum(int(d[i:i+2],16) for i in range(1, len(d), 2))
    cs = (256 - (cs & 0xff))&0xff
    d += "{:02x}\n".format(cs)
    #print(d, end="")
    s.write(d.encode())
    if t == 0:
        echo = s.readline().decode()
        if echo.upper() != d.upper(): print("ERROR \n      wrote {}\n   received {}".format(d.strip(), echo.strip()))

base_address = 0
with open(fname) as f:
    nrec = 0
    for record in f:
        nrec += 1
        length = int(record[1:3], 16)
        if record[7:9] == "02":
            base_address = 16*int(record[9:13], 16)
        elif record[7:9] == "04":
            base_address = int(record[9:13], 16)<<16
        elif record[7:9] == "00":
            address = base_address + int(record[3:7], 16)
            for i in range(length):
                if (address & 0xfffffff8) not in dwords:
                    dwords[(address & 0xfffffff8)] = bytearray([255]*8)
                dwords[(address & 0xfffffff8)][address & 0x07] = int(record[9+2*i:11+2*i], 16)
                address += 1
    # Exclude bootloader code from the uploaded program
    dword_addresses = [x for x in sorted(dwords.keys()) if x < 0x0807c000]
    base_address = 0
    start_address = 0
    for i, a in enumerate(dword_addresses):
        if len(buf) == 0:
            start_address = a
            buf = dwords[a]
        elif a + 8 - start_address <= 64:
            buf += bytearray([255]*(a - start_address - len(buf)))
            buf += dwords[a]
        else:
            write_record(0, (start_address - 0x08000000)>>3, buf);
            start_address = a
            buf = dwords[a]

        if len(buf) == 64:
            write_record(0, (start_address - 0x08000000)>>3, buf);
            buf = bytearray([])
        print("\rWritten {}/{} doublewords".format(i+1, len(dword_addresses)), end="")
    if buf:
        write_record(0, (start_address - 0x08000000)>>3, buf)
print("\nResetting")
reset()

