#!/usr/bin/python

import sys
import os

f1 = open(sys.argv[1], 'r')
f2 = open(sys.argv[2], 'w')
for line in f1:
  f2.write(line.replace(sys.argv[3], os.getenv('ROBOT')))
f1.close()
f2.close()
