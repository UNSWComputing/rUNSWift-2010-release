#!/usr/bin/python

import sys, os, re

def getdirs(dir, result):
   for name in os.listdir(dir):
      full = os.path.join(dir, name)
      if os.path.isdir(full) and name != "build":
         m = re.match(r'^.*/([^/]+)$', full)
         result.append(m.group(1))
         getdirs(full, result)

dir = sys.argv[2]
dirs = ['robot']
getdirs(dir, dirs)
data = {
      "numlogfiles" : len(dirs),
      "logfiles" : ",\n".join(["      "+"path + \"/"+dir+".log\"" for dir in dirs])
      }

sys.stdout.write(open(sys.argv[1], 'r').read() % data)

