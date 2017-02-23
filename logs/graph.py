#!/usr/bin/env python
"""
Copy ALL the logs from the roborio to this directory
Read the latest log and grab the last |TIME_WINDOW| seconds of data
Create a graph
"""
import numpy as np
import matplotlib.pyplot as plt
import csv, os

TEAM_NUMBER = 973

time_window = 10 #30 seconds in miliseconds
points = []

TIME_COL = 7
DATA_COLS = [12, 13, 14]

os.system('scp lvuser@roborio-%d-frc.local:/home/lvuser/*.txt .' % TEAM_NUMBER)
latest = sorted([f for f in os.listdir() if f[:4] == "log:"])[-1]

with open(latest, "r") as f:
    reader = csv.reader(f, delimiter=',')
    reader.__next__() # throw away title row
    for row in reader:
        points.append(
                tuple(
                    [float(row[TIME_COL])] +
                    [float(row[x]) for x in DATA_COLS]))

max_time = points[-1][0]
min_time = max_time - time_window
print(max_time)
print(min_time)

x1 = [p[0] for p in points if p[0] >= min_time]
y1 = [p[1] for p in points if p[0] >= min_time]
y2 = [p[2] for p in points if p[0] >= min_time]
y3 = [p[3] for p in points if p[0] >= min_time]

plt.subplot(2, 1, 1)
plt.plot(x1, y1, 'o-')
plt.plot(x1, y2, 'o-')
plt.plot(x1, y3, 'o-')
plt.title('A tale of 2 subplots')
plt.ylabel('Damped oscillation')

plt.show()
