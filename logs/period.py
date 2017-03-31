"""
Determine the average loop period during enabled state
"""

import sys

period = []
sum = 0
count = 1
print("Running script")

with open(sys.argv[1]) as file:
    try:
        for row in csv.reader(f, delimiter=","):
            period.append(row[0])
    except:
        pass
    for x in period:
        sum += x
        count += 1
    average = sum / count
    print(period)
print(average / count)
