#!/usr/bin/env python
"""
Copy ALL the logs from the roborio to this directory
Read the latest log and grab the last |TIME_WINDOW| seconds of data
Create a graph
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv, os

TEAM_NUMBER = 973

time_window = 40 #time in seconds
points = []

TIME_COL = 0
#DATA_COLS = [17, 19, 18, 20] #flywheel tuning
#DATA_COLS = [4, 5, 2, 9]
fields = [
        "linear vel incr goal", "linear vel incr actual",
        "angular vel incr goal", "Angular Rate",
        "linear pos incr goal", "linear pos incr actual",
        "angular pos incr goal", "angular pos incr actual"]
DATA_COLS = []
titles = []

os.system('scp -p lvuser@roborio-%d-frc.local:/home/lvuser/*.txt .' % TEAM_NUMBER)
files = sorted([(os.path.getmtime(f), f) for f in os.listdir(".") if f[:4] == "log:"])
latest = files[-1]

print(latest)

print(files)

with open(latest[1], "r") as f:
    reader = csv.reader(f, delimiter=',')
    titles = reader.__next__()
    for field in fields:
        print(field)
        DATA_COLS.append(titles.index(field))
    print(DATA_COLS)

    print(list(zip(titles, range(999)))) # throw away title row
    for row in reader:
        try:
            points.append(
                    tuple(
                        [float(row[TIME_COL])] +
                        [float(row[x] or 0) for x in DATA_COLS]))
        except Exception as e:
            print(e)
            print(row)
            print("Misread row")

max_time = points[-1][0]
min_time = max_time - time_window
print(max_time)
print(min_time)

x1 = [p[0] for p in points if p[0] >= min_time]
y1 = [p[1] for p in points if p[0] >= min_time]
y2 = [p[2] for p in points if p[0] >= min_time]
y3 = [p[3] for p in points if p[0] >= min_time]
y4 = [p[4] for p in points if p[0] >= min_time]
y5 = [p[5] for p in points if p[0] >= min_time]
y6 = [p[6] for p in points if p[0] >= min_time]
y7 = [p[7] for p in points if p[0] >= min_time]
y8 = [p[8] for p in points if p[0] >= min_time]

plt.subplot(2, 2, 1)
plt.xlabel('time')

line1, = plt.plot(x1, y1, '-', color='red')
line2, = plt.plot(x1, y2, '-',color="blue")

plt.subplot(2, 2, 2)
line3, = plt.plot(x1, y3, '-')
line4, = plt.plot(x1, y4, '-')
#plt.ylim([2500, 3500])

plt.subplot(2, 2, 3)
plt.title('A tale of 2 subplots')

line5, = plt.plot(x1, y5, '-')
line6, = plt.plot(x1, y6, '-')

plt.subplot(2, 2, 4)
line7, = plt.plot(x1, y7, '-')
line8, = plt.plot(x1, y8, '-')
plt.ylabel('Damped oscillation')
plt.xlabel('time')

plt.show()
