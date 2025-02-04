import socket
import time
import struct
import sys
import select

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("192.168.72.1", 40000))


def parse_response(print_response=False):
    frame = s.recv(1500)
    i = 0
    frames = []
    while i < len(frame)-1:
        if frame[i:i+2] == b"\x55\xff":
            bus, length, id = struct.unpack("<BBI", frame[i+2:i+8])
            packet = {"bus": bus, "id": id & 0x1fffffff, "data": frame[i+8:i+4+(length&0x7f)]}
            if print_response: print("bus {:02x}, id {}/{:08x} (device {})".format(bus, "EXT" if (id & (1<<30)) else "STD", id & 0x1fffffff, (id >> 18) & 0x7ff))
            if (id & (1<<30)):
                if (id & (1<<16)):
                    packet["type"] = "data"
                    if print_response: print("    Data", frame[i+8:i+4+(length&0x7f)])
                else:
                    stat = struct.unpack("<BBHI", frame[i+8:i+16])
                    packet["type"] = "status"
                    packet["status"], packet["bankstatus"], packet["flashstatus"], packet["bootstate"] = stat
                    if print_response: print("    Status {:02x}, bank status {:02x}, FLASH status {:04x}, boot state {:08x}".format(*stat))
            frames.append(packet)
            i += (length & 0x7f)+4
    return frames


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

def read_data(bus, id, address, length, bankmode):
    frame = bytearray([0x55, 0xff, bus, 0x80 | (5)])+struct.pack("<IBB", (1<<30) | (id<<18) | address | (1<<17) | (1<<16), length, bankmode)
    s.sendto(frame, ('192.168.72.100', 5001))
    return parse_response(False)



t = time.time()
#frame = bytearray([0x55, 0xff, 0x01, 0x80 | 12])+struct.pack("<I", (1<<30) | (4<<18))+(b"\x55"*8)
#frame = bytearray([0x55, 0xff, 0x01, 0x00 | 6])+struct.pack("<I", 123 | (1<<30))+(b"\xaa\xff")

# Enter the bootloader
def boot(id):
    print("Entering bootloader", id)
    send_command(1, id, b"\x55"*8)
    parse_response(True)

def boot_enum():
    send_command(1, 0x7ff, b"\x55"*8)
    frames = []
    rlist, wlist, xlist = select.select([s], [], [], 0.1)
    while rlist:
        frames += parse_response(True)
        rlist, wlist, xlist = select.select([s], [], [], 0.1)
    print("Timed out waiting for IDs")
    ids = {}
    for packet in frames:
        if "type" in packet and packet["type"] == "status":
            ids[(packet["id"]>>18) & 0x7f] = packet
    return ids

def reset(id):
    send_command(1, id, b"\x00")

def program(id, fname):
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
                send_data(1, id, (start_address - 0x08000000)>>3, buf);
                start_address = a
                buf = dwords[a]

            if len(buf) == 64:
                #write_record(0, (start_address - 0x08000000)>>3, buf);
                send_data(1, id, (start_address - 0x08000000)>>3, buf);
                buf = bytearray([])
            print("\rWritten {}/{} doublewords".format(i+1, len(dword_addresses)), end="")
        if buf:
            #write_record(0, (start_address - 0x08000000)>>3, buf)
            send_data(1, id, (start_address - 0x08000000)>>3, buf);
    print("\nResetting")
    softswap(id)

def softswap(id):
    send_command(1, id, bytearray([0x01]))

def verify(id):
    send_command(1, id, b"\x02")
    parse_response(True)

def hardswap(id):
    send_command(1, id, b"\x03")
    parse_response(True)

if len(sys.argv) > 2:
    id = int(sys.argv[1])
    cmd = sys.argv[2]
    if cmd == "program":
        if len(sys.argv) > 3:
            fname = sys.argv[3]
            boot(id)
            program(id, fname)
            time.sleep(1)
            boot(id)
            verify(id)
            hardswap(id)
    elif cmd == "boot":
        boot(id)
    elif cmd == "enum":
        boot(id)
    elif cmd == "read":
        boot(id)
        read_data(1, id, 0x3ffe0>>3, 32, 0)
    elif cmd == "ls":
        ids = boot_enum()
        for id in ids:
            print("Found ID", id, ids[id])
            resp = read_data(1, id, 0x3ffe0>>3, 32, ids[id]["bankstatus"]&1)[0]
            if "type" in resp and resp["type"] == "data":
                print("    Bank 1:", resp["data"].strip(b"\x00").decode())
            resp = read_data(1, id, 0x3ffe0>>3, 32, 1 ^ (ids[id]["bankstatus"]&1))[0]
            if "type" in resp and resp["type"] == "data":
                print("    Bank 2:", resp["data"].strip(b"\x00").decode())
        reset(0x7ff)


s.close()

