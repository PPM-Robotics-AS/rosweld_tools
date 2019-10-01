#!/usr/bin/env python
#how many times consequtively the process needs to exceed the limit
POINTS_TO_KILL = 3
#limit of processor usage percent to get a point
CPU_USE_FOR_POINT = 50
#process names, cmdlines, filenames to look for
PROCESS_NAMES = ["python", "bash"]
#how many seconds to wait between two cycles
CYLCE_TIME = 3

import os, psutil, datetime, time

#searches for processes with names, cmdlines, filenames contained by PROCESS_NAMES
def find_procs_by_name(names):
    result = []
    for p in psutil.process_iter():
        name_, exe, cmdline = "", "", []
        try:
            name_ = p.name()
            cmdline = p.cmdline()
            exe = p.exe()
        except (psutil.AccessDenied, psutil.ZombieProcess):
            pass
        except psutil.NoSuchProcess:
            continue
        if name_ in names or cmdline[0] in names or os.path.basename(exe) in names:
            result.append(p)
    return result

points = {}
while True:
    print "***********************"
    #searches for processes with names, cmdlines, filenames contained by PROCESS_NAMES
    procs = find_procs_by_name(PROCESS_NAMES)
    for proc in procs:
        #make a dictionary entry for the pid
        if points.get(proc.pid) == None:
            points[proc.pid] = 0

        #measure cpu time
        cpu = proc.cpu_percent(0)

        print proc, ", proc usage:", cpu, ", points:", points[proc.pid]

        #if the cpu usage is more than CPU_USE_FOR_POINT%, then it gets a point
        if (cpu > CPU_USE_FOR_POINT):
            points[proc.pid] += 1

            #if a proc has more, than POINTS_TO_KILL consequent points, it gets killed
            if (points[proc.pid] >= POINTS_TO_KILL):
                proc.kill()
                print "killed"

        else:
            #if the process has less, than CPU_USE_FOR_POINT% proc usage, the points are cleared
            points[proc.pid] = 0

    #sleep for CYLCE_TIME seconds
    time.sleep(CYLCE_TIME)