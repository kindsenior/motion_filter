import os
import re
import sys

if len(sys.argv) < 2:
    print "less argments"

def time_scale (fname,speed, robotname="HRP2JSK"):
    speed = float(speed)
    path = os.environ.get("HOME")+"/"+os.environ.get("CNOID_WORKSPACE")+"/"+robotname+"/"+fname+"/"
    ifname = path + fname + ".pseq"

    ofname = path + fname + "_x" + str(int(speed*100)) + ".pseq"
    of = open(ofname, 'w')
    
    for line in open(ifname,'r'):
        if re.search("time",line):
            time = line.replace('time: ','')
            # print time
            line = "    time: " + str(float(time[:-1])/speed) + "\n"
            # print float(time[:-1])
            # print float(time[:-1])/speed
        of.write(line)

    of.close()

time_scale(sys.argv[1], sys.argv[2])

