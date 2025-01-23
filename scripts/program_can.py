import socket
import time
import struct
import sys

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("192.168.72.1", 40000))

fname = sys.argv[1]

def print_response():
    frame = s.recv(1500)
    i = 0
    while i < len(frame)-1:
        if frame[i:i+2] == b"\x55\xff":
            bus, length, id = struct.unpack("<BBI", frame[i+2:i+8])
            print("bus {:02x}, id {}/{:08x}".format( bus, "EXT" if (id & (1<<30)) else "STD", id & 0x1fffffff))
            if (id & (1<<30)):
                if (id & (1<<16)):
                    print("    Data", frame[i+8:i+4+(length&0x7f)])
                else:
                    stat = struct.unpack("<BBHI", frame[i+8:i+16])
                    print("    Status {:02x}, bank status {:02x}, FLASH status {:04x}, boot state {:08x}".format(*stat))
                    if stat[0] == 21: print_response()
            i += (length & 0x7f)+4


def send_command(bus, id, data):
    frame = bytearray([0x55, 0xff, bus, 0x80 | (len(data)+4)])+struct.pack("<I", (1<<30) | (id<<18))+data
    s.sendto(frame, ('192.168.72.100', 5001))

def send_data(bus, id, address, data):
    l = len(data)
    if l >= 32 and l&8:
        address |= (1<<15)
        l += 8
    frame = bytearray([0x55, 0xff, bus, 0x80 | (len(data)+4)])+struct.pack("<I", (1<<30) | (id<<18) | address | (1<<17))+data
    s.sendto(frame, ('192.168.72.100', 5001))
    echo = s.recv(1500)
    if echo[8:] != frame[8:]:
        print("ERROR")
        print("    "+"".join(hex(x)[2:].rjust(2, "0") for x in frame[8:]))
        print("    "+"".join(hex(x)[2:].rjust(2, "0") for x in echo[8:]))


t = time.time()
#frame = bytearray([0x55, 0xff, 0x01, 0x80 | 12])+struct.pack("<I", (1<<30) | (4<<18))+(b"\x55"*8)
#frame = bytearray([0x55, 0xff, 0x01, 0x00 | 6])+struct.pack("<I", 123 | (1<<30))+(b"\xaa\xff")

# Enter the bootloader
send_command(1, 4, b"\x55"*8)
print_response()

base_address = 0
dwords = {}
buf = bytearray([])
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
    dword_addresses = [x for x in sorted(dwords.keys())]
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
            #write_record(0, (start_address - 0x08000000)>>3, buf);
            send_data(1, 4, (start_address - 0x08000000)>>3, buf);
            start_address = a
            buf = dwords[a]

        if len(buf) == 64:
            #write_record(0, (start_address - 0x08000000)>>3, buf);
            send_data(1, 4, (start_address - 0x08000000)>>3, buf);
            buf = bytearray([])
        print("\rWritten {}/{} doublewords".format(i+1, len(dword_addresses)), end="")
    if buf:
        #write_record(0, (start_address - 0x08000000)>>3, buf)
        send_data(1, 4, (start_address - 0x08000000)>>3, buf);
print("\nResetting")
send_command(1, 4, bytearray([0x01]))
time.sleep(1)
# Enter the bootloader
print("Entering bootloader")
send_command(1, 4, b"\x55"*8)
print_response()
send_command(1, 4, b"\x02")
print_response()
send_command(1, 4, b"\x03")
print_response()
s.close()
