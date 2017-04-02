"""
Determine the average loop period during enabled state
"""

import sys, csv

period = []
diff = 0.0
sum = 0.0
count = 0.0
print("Running script")

with open(sys.argv[1]) as f:
    try:
        for row in csv.reader(f, delimiter=","):
            period.append(row[0])
    except:
        pass
    period.pop(0) #Take out "Time" cell
    for x in period:
        diff = float(x) - diff
        sum = sum + diff
        count = count + 1
    print(period)
    print(sum, diff, count)
    average = diff / count
print(average)
