import socket
import time
import struct
import sys
import select

BOOT_STATUS_OK =                0x00
BOOT_STATUS_INVALID_ADDRESS =   0x01
BOOT_STATUS_ERASE_ERROR =       0x02
BOOT_STATUS_PROG_ERROR =        0x03
BOOT_STATUS_STATE_ERROR =       0x04
BOOT_STATUS_NB_ERROR =          0x05
BOOT_STATUS_ALREADY_BOOTED =    0x06
BOOT_STATUS_NO_BSM =            0x07
BOOT_STATUS_SOFTSWAP_SUCCESS =  0x08
BOOT_STATUS_MAINBANK =          0x09

extmode_addr = 0

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("192.168.72.1", 40000))
# Flush waiting messages
rlist, wlist, xlist = select.select([s], [], [], 0)
while rlist:
    s.recv(1500)
    rlist, wlist, xlist = select.select([s], [], [], 0)


def parse_response(print_response=False, timeout=0.5, mintime=0, print_data=False):
    frames = []
    tstart = time.time()
    rlist, wlist, xlist = select.select([s], [], [], timeout)
    while rlist:
        frame = s.recv(1500)
        i = 0
        while i < len(frame):
            bus, length, ts, id = struct.unpack("<BBHI", frame[i:i+8])
            if id & (1<<30) and bus in [1, 2, 3]:
                packet = {"bus": bus, "id": id, "data": frame[i+8:i+8+(length&0x7f)], "fd": length>>7, "board": (id>>18)&0x7f}
                if (packet["id"] & (1<<30)):
                    if (packet["id"] & (1<<16)):
                        packet["type"] = "data"
                        if print_data: print("    Data", packet["data"])
                    else:
                        if print_response: print("bus {:02x}, id {}/{:08x} (device {})".format(packet["bus"], "EXT" if (packet["id"] & (1<<30)) else "STD", packet["id"] & 0x1fffffff, packet["board"]))
                        stat = struct.unpack("<BBHI", packet["data"])
                        packet["type"] = "status"
                        packet["status"], packet["bankstatus"], packet["flashstatus"], packet["bootstate"] = stat
                        if print_response: print("    Status {:02x}, bank status {:02x}, FLASH status {:04x}, boot state {:08x}".format(*stat))
                frames.append(packet)
            if bus == 5:
                if id == 1:
                    ecr1, psr1, ecr2, psr2, ecr3, psr3, arb_e, data_e, bus_off = struct.unpack("<IIIIIIHHH", frame[i+8:i+8+(length&0x7f)])
                    if print_response:
                        print("FDCAN1 ECR: {:08x}, PSR: {:08x}".format(ecr1, psr1))
                        print("FDCAN2 ECR: {:08x}, PSR: {:08x}".format(ecr2, psr2))
                        print("FDCAN3 ECR: {:08x}, PSR: {:08x}".format(ecr3, psr3))
                        print("Arbitration errors: {}, data errors: {}, bus-off: {}".format(arb_e, data_e, bus_off))
                    frames.append({"bus": 5, "data": frame[i+8:i+8+(length&0x7f)], "id": id, "errors": {"arb": arb_e, "data": data_e, "bus_off": bus_off}})
                else:
                    if print_response: print("Control packet", frame[i+8:i+8+(length&0x7f)])
                    frames.append({"bus": 5, "data": frame[i+8:i+8+(length&0x7f)], "id": id})
            #elif bus == 1:
            #    print("rejecting bad packet", bus, id)
            i += (length & 0x7f)+8
        t = time.time()
        if (frames and (t - tstart) >= mintime) or (t - tstart) >= timeout:
            break
        rlist, wlist, xlist = select.select([s], [], [], timeout - (t - tstart))
    return frames


def send_command(bus, id, data):
    frame = struct.pack("<BBHI", bus, 0x80 | (len(data)), 0, (1<<30) | (id<<18))+data
    s.sendto(frame, ('192.168.72.100', 5001))

def send_data(bus, id, address, data):
    l = len(data)
    if l >= 32 and l&8:
        address |= (1<<15)
        l += 8
    resp = []
    while not resp:
        frame = struct.pack("<BBHI", bus, 0x80 | (len(data)), 0, (1<<30) | (id<<18) | address | (1<<17))+data
        s.sendto(frame, ('192.168.72.100', 5001))
        resp = parse_response(True)
        for p in resp:
            if p["type"] == "data":
                if p["data"] != frame[8:]:
                    print("ERROR")
                    print("    "+"".join(hex(x)[2:].rjust(2, "0") for x in frame[8:]))
                    print("    "+"".join(hex(x)[2:].rjust(2, "0") for x in p["data"]))

def read_data(bus, id, address, length, bankmode):
    print("Reading from", id, address, length)
    frame = struct.pack("<BBHIBB", bus, 0x83, 0, (1<<30) | (id<<18) | address | (1<<17) | (1<<16), length, bankmode)
    s.sendto(frame, ('192.168.72.100', 5001))
    return parse_response(True, print_data=True)



t = time.time()
#frame = bytearray([0x55, 0xff, 0x01, 0x80 | 12])+struct.pack("<I", (1<<30) | (4<<18))+(b"\x55"*8)
#frame = bytearray([0x55, 0xff, 0x01, 0x00 | 6])+struct.pack("<I", 123 | (1<<30))+(b"\xaa\xff")

# Enter the bootloader
def boot(id, pr=True):
    if pr: print("Entering bootloader", id)
    send_command(1, id, b"\x55"*8)
    resp = parse_response(pr)
    if not resp and not pr:
        print("No response received")
        return False
    if resp[0]["status"] == BOOT_STATUS_MAINBANK:
        parse_response(pr)
    if resp[0]["status"] == BOOT_STATUS_SOFTSWAP_SUCCESS:
        parse_response(pr)
    return True

def boot_enum():
    print("Booting all")
    send_command(1, 0x7ff, b"\x55"*8)
    print("Collecting IDs ... ", end="")
    frames = parse_response(False, timeout=0.5, mintime=0.5)
    print("done")
    ids = {}
    for packet in frames:
        if "type" in packet and packet["type"] == "status":
            ids[(packet["id"]>>18) & 0x7f] = packet
    return ids

def reset(id):
    print("Resetting", id)
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
    print("Soft swapping", id)
    send_command(1, id, b"\x01")
    parse_response(True, timeout=0.5)

def verify(id):
    print("Verifying", id)
    send_command(1, id, b"\x02")
    parse_response(True)

def hardswap(id):
    print("Hard swapping", id)
    send_command(1, id, b"\x03")
    parse_response(True)

def extmode(id, enable):
    if enable:
        print("Entering external programming mode for", id)
        send_command(1, id, b"\x04")
    else:
        print("Leaving external programming mode for", id)
        send_command(1, id, b"\x05")
    parse_response(True)

def extmode_read(bus, id, address, length, erase=0):
    global extmode_addr
    print("Reading from", id, address, length)
    frame = struct.pack("<BBHIBHxI", bus, 0x88, 0, (1<<30) | (id<<18) | (address & 0x7fff) | (1<<17) | (1<<16), length, address >> 15, erase)
    s.sendto(frame, ('192.168.72.100', 5001))
    extmode_addr = (address & 0xffff8000)
    if length:
        return parse_response(True, print_data=True)

def extmode_write(bus, id, address, data):
    global extmode_addr
    if (address & 0xffff8000) != extmode_addr:
        extmode_read(bus, id, address, 0)
    frame = struct.pack("<BBHI", bus, len(data) | 0x80, 0, (1<<30) | (id<<18) | (address & 0x7fff) | (1<<17)) + data
    s.sendto(frame, ('192.168.72.100', 5001))
    return parse_response(True, print_data=True)

def query_errors(pr=True):
    s.sendto(b"\x05\x00\x00\x00\x01\x00\x00\x00", ('192.168.72.100', 5001))
    return parse_response(pr)[0]["errors"]

def transmit_check_error(bus, id, data, nit, boot=False):
    initial = query_errors(False)
    lost = 0
    for x in range(nit):
        s.sendto(struct.pack("<BBHI", bus, 0x80 | len(data), 0, id)+data+b"\x05\x00\x00\x00\x01\x00\x00\x00", ('192.168.72.100', 5001))
        bootresp = parse_response(False)
        resp = bootresp[0]["errors"]
        if boot:    
            if len(bootresp) < 2: 
                bootresp = parse_response(False, timeout=0.1)
            else:
                bootresp = bootresp[1:]
            if bootresp:
                if bootresp[0]["status"] == BOOT_STATUS_MAINBANK:
                    parse_response(False)
                if bootresp[0]["status"] == BOOT_STATUS_SOFTSWAP_SUCCESS:
                    parse_response(False)
            else: lost += 1
        pct_prog = 100 * (x + 1) / nit
        pct_arb = 100 * (resp["arb"] - initial["arb"]) / (x + 1)
        pct_data = 100 * (resp["data"] - initial["data"]) / (x + 1)
        pct_bo = 100 * (resp["bus_off"] - initial["bus_off"]) / (x + 1)
        pct_lost = 100 * lost / (x + 1)
        print("\33[2K\rProgress: {:1.1f}%, arbitration errors: {:1.1f}%, data errors: {:1.1f}%, bus off: {:1.1f}%, lost: {:1.1f}%".format(pct_prog, pct_arb, pct_data, pct_bo, pct_lost), end="")
    print()


cmd = sys.argv[1]
if cmd == "program":
    id = int(sys.argv[2])
    if len(sys.argv) > 3:
        fname = sys.argv[3]
        boot(id)
        program(id, fname)
        boot(id)
        verify(id)
        hardswap(id)
elif cmd == "boot":
    boot(int(sys.argv[2]))
elif cmd == "softswap":
    id = int(sys.argv[2])
    boot(id)
    softswap(id)
elif cmd == "hardswap":
    id = int(sys.argv[2])
    boot(id)
    softswap(id)
    boot(id)
    verify(id)
    hardswap(id)
elif cmd == "read":
    id = int(sys.argv[2])
    boot(id)
    read_data(1, id, 0x3ffe0>>3, 32, 0)
elif cmd == "ls":
    if len(sys.argv) > 2:
        id = int(sys.argv[2])
        send_command(1, id, b"\x55"*8)
        ids = {id: parse_response(False, timeout=0.5, mintime=0.5)[0]}
    else:
        ids = boot_enum()
    for id in ids:
        print("Found ID", id, ids[id])
        resp = read_data(1, id, 0x3ffe0>>3, 32, ids[id]["bankstatus"]&1)[0]
        if "type" in resp and resp["type"] == "data":
            print("    Bank 1:", resp["data"].strip(b"\x00"))
        resp = read_data(1, id, 0x3ffe0>>3, 32, 1 ^ (ids[id]["bankstatus"]&1))[0]
        if "type" in resp and resp["type"] == "data":
            print("    Bank 2:", resp["data"].strip(b"\x00"))
    reset(0x7ff)
elif cmd == "reset":
    reset(0x7ff)
elif cmd == "txoff":
    s.sendto(b"\x05\x01\x00\x00\x00\x00\x00\x00\x00", ('192.168.72.100', 5001))
elif cmd == "txoff_retry":
    while True:
        s.sendto(b"\x05\x01\x00\x00\x00\x00\x00\x00\x00", ('192.168.72.100', 5001))
        if parse_response(timeout=0.5): break
        else: print("TXOFF failed, trying agin...")
elif cmd == "txon":
    s.sendto(b"\x05\x01\x00\x00\x00\x00\x00\x00\x01", ('192.168.72.100', 5001))
elif cmd == "c70off":
    frame = struct.pack("<BBHI", 2, 0x08, 0, 0x701)+b"\xff"*8
    s.sendto(frame, ('192.168.72.100', 5001))
elif cmd == "c70on":
    frame = struct.pack("<BBHI", 2, 0x08, 0, 0x701)+b"\x00"*8
    s.sendto(frame, ('192.168.72.100', 5001))
elif cmd == "extmode":
    id = int(sys.argv[2])
    boot(id)
    extmode(id, True)
    extmode_read(1, id, 0x20000, 32)
    extmode_read(1, id, 0x20000, 32, erase=0x30000)
    with open(sys.argv[3], "rb") as f:
        for i in range(0, 0x30000, 64):
            data = f.read(64)
            address = 0x20000 + i
            if (address & 0xffff8000) != extmode_addr:
                extmode_read(1, id, address, 0)
                extmode_addr = (address & 0xffff8000)
            send_data(1, id, address & 0x7fff, data)
            print("\rWritten {:08x}/00030000 bytes".format(i+64), end="")
        print()
elif cmd == "errors":
    query_errors()
elif cmd == "test":
    ids = boot_enum()
    print(ids)
    # Test each bus
    for bus in [1, 2, 3]:
        print("Testing bus {}".format(bus))
        transmit_check_error(bus, 1<<30, b"", 1000)
        query_errors()
    # Test each board
    for board in ids:
        print("Testing board {}".format(board))
        transmit_check_error(1, (1<<30) | (board << 18), b"\x55"*8, 100, True)
        #for x in range(1000):
        #    boot(board, False)
        query_errors()
    reset(0x7ff)
elif cmd == "vectornav":
    # Program VectorNAV via the rear SSDB
    id = int(sys.argv[2])
    boot(id)
    extmode(id, True)
    send_data(1, id, 0, b"text\n")
    reset(id)

s.close()

