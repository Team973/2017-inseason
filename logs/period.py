"""
Determine the average loop period during enabled state
"""

import sys, csv

period = []
rows = []
state = []
diff = 0.0
sum = 0.0
count_p = 0
count_s = 0
print("Running script")

with open(sys.argv[1]) as f:
    try:
        for row in csv.reader(f, delimiter=","):
            rows.append(row)
            period.append(row[0])
            state.append(row[22])
    except:
        pass
    period.pop(0) #Take out "Time" cell
    while count_p < len(period) - 1:
        diff = float(period[count_p + 1]) - float(period[count_p])
        sum = sum + diff
        count_p = count_p + 1
        print(diff)
    print(sum, count_p)
    average = sum / float(count_p)
    print(average)

    state.pop(0)   #Take out "Auto state" cell
    print(",".join([str(x) for x in rows[0]]))
    while count_s < len(state) - 1:
        if state[count_s + 1] > state[count_s]:
            print(",".join([str(x) for x in rows[count_s]]))
            print(",".join([str(x) for x in rows[count_s + 1]]))
        count_s += 1
