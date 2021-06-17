import csv

filename = "./Bounce.path"

output = "";

with open(filename, newline='') as csvfile:
    reader = csv.reader(csvfile)
    startedFlag = False
    for row in reader:
        if(not startedFlag):
            startedFlag = True
            continue
        x = row[0]
        y = float(row[1])
        output += ("new Waypoint(" + str((y + 4.572)) + ", " + x + ", 0),\n")
print(output)