import os
import re

src_dir = "./src/driver/Inc"
mock_dir = "./src/mock"

excluded = ["typedef", "extern"]

for root, dir, files in os.walk(src_dir):
    for name in files:
        l = ""
        os.system(f"cp {src_dir}/{name} {mock_dir}/Inc/{name}")
        with open(f"{mock_dir}/Inc/{name}", "r") as file:
            for line in file:
                x = re.search("^.+ .+\(.*\);$", line)
                if (x != None):
                    l += "__weak " + x.string[:len(x.string)-2] + " {}\n"

        src = open(f"{mock_dir}/Src/{name[:len(name)-1]}c", "w")
        src.write(f"#include {name}\n\n")
        src.write(l)
        src.close()

