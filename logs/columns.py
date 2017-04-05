import os, csv

latest = sorted([f for f in os.listdir(".") if f[:4] == "log:"])[-1]

with open(latest, "r") as f:
    reader = csv.reader(f, delimiter=',')
    titles = reader.__next__()
    for i in list(zip(titles, range(999))):
        print(i)

