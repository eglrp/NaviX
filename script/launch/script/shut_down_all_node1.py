#!/usr/bin/env python
import os

print 'this script will kill all the rosnode'
print '`````````````````````````````````````'
nodes = os.popen("rosnode list").readlines()
for i in range(len(nodes)):
    nodes[i] = nodes[i].replace("\n","")

for node in nodes:
    	os.system("rosnode kill "+ node)
print '`````````````````````````````````````'
print 'kill all down!'
