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

time_window = 20 #time in seconds
points = []

TIME_COL = 0
DATA_COLS = [17, 19, 18, 20] #flywheel tuning
#DATA_COLS = [5, 4, 7, 10]
GRAPHS = [["Left Encoder Rate", "Left motor signal (pow or vel)"], ["Left motor voltage"]]
titles = []

os.system('scp lvuser@roborio-%d-frc.local:/home/lvuser/*.txt .' % TEAM_NUMBER)
latest = sorted([f for f in os.listdir(".") if f[:4] == "log:"])[-1]

with open(latest, "r") as f:
    reader = csv.reader(f, delimiter=',')
    titles = reader.__next__()
    print(list(zip(titles, range(999)))) # throw away title row
    for row in reader:
        try:
            points.append(
                    tuple(
                        [float(row[TIME_COL])] +
                        [float(row[x]) for x in DATA_COLS]))
        except:
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

plt.subplot(2, 1, 1)
plt.plot(x1, y1, 'o-', label="speed")
plt.plot(x1, y2, 'o-', label="setpoint")
plt.ylim([2200, 3700])
plt.subplot(2, 1, 2)
plt.plot(x1, y3, 'o-', label="setpoint")
plt.plot(x1, y4, 'o-', label="setpoint")
plt.title('A tale of 2 subplots')
plt.ylabel('Damped oscillation')

plt.show()
