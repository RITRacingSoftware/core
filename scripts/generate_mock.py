import os
import re

src_dir = "./src/driver/Inc"
mock_dir = "./src/mock"

os.system(f"mkdir {mock_dir}/Inc")
os.system(f"mkdir {mock_dir}/Src")

for root, dir, files in os.walk(src_dir):
    for name in files:
        if (name == "rtt.h"): continue
        pragma = ""
        body = ""
        os.system(f"cp -p {src_dir}/{name} {mock_dir}/Inc/{name}")
        with open(f"{mock_dir}/Inc/{name}", "r") as file:
            print(name) 
            for line in file:
                if (line[0] == '#'): continue
                arr = line.split()
                if ("static" in arr): continue
                x = re.search("[^ \*\)]+\(.*\)", line)
               if (x == None): continue
                part = x.group(0)
                full = x.string
                pragma += f"#pragma weak {part[:part.index('(')]}\n" 
                body += full[:len(full)-2] + " {}\n"

        src = open(f"{mock_dir}/Src/{name[:len(name)-1]}c", "w")
        src.write(f'#include "{name}"\n\n')
        src.write(f"{pragma}\n")
        src.write(body)
        src.close()
