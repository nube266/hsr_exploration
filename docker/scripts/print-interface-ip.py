#! python

################################################################################

import argparse
import subprocess
import sys
import ipaddress

parser = argparse.ArgumentParser(description="""Search an IP address of the current host for communicating with the given address.""")
parser.add_argument('address', nargs=1, help='an IP address')
args = parser.parse_args()
target = ipaddress.ip_address(unicode(args.address[0]))

lines = subprocess.check_output(["ip", "route", "list", "scope", "link"]).split('\n')

# Extract the 1st, 3rd, and 9th components from lines consisting of 9 or more elements.
pairs = [(x[0],x[2],x[8]) for x in [v.split(' ') for v in lines] if 9 <= len(x)]
found = False
for net, interface, address in pairs:
    if target in ipaddress.ip_network(unicode(net)):
        print(address)
        found = True
        break

if not found:
    sys.exit(1)
