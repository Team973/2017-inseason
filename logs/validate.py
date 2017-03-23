#!/usr/bin/env python
"""
Copy ALL the logs from the roborio to this directory
Read the latest log and grab the last |TIME_WINDOW| seconds of data
Create a graph
"""
import csv, os

matches = []

for fName in [f for f in os.listdir(".") if f[:9] != "log::2017"]:
    with open(fName, "r") as f:
        print(os.path.getmtime(fName))
        statesSeen = set()
        for row in csv.reader(f, delimiter=","):
            statesSeen.add(row[16])

        print(statesSeen)
        if statesSeen == set(["TeleOp", "Game State", "Disabled", "Auto"]):
            matches.append((os.path.getmtime(fName), fName))

print(sorted(matches))
