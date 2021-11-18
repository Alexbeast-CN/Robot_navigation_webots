# A python code to build map
import numpy as np
import csv

# The x and y length of the map
x = 92
y = 92

# Build the surrounding wall for the mpa
a = np.zeros((x,y))

for i in range(np.size(a,1)):
    a[0][i] = 1
    a[np.size(a,0)-1][i] = 1

for i in range(np.size(a,0)):
    a[i][0] = 1
    a[i][np.size(a,1)-1] = 1

# Create obstacles for the map

for i in range(30):
    for j in range(30):
        a[30+i][20+j] = 1

for i in range(10):
    for j in range(20):
        a[40+i][20+j] = 0


# Export the map into a csv file

# with open('easyMap.csv', 'w', encoding='UTF8') as f:
#     writer = csv.writer(f)

#     # write the data
#     writer.writerow(a)

np.savetxt('EasyMap1.csv', a, delimiter = ',')