import numpy as np

a = np.zeros((10,10))

for i in range(10):
    a[0][i] = 1
    a[i][0] = 1
    a[9][i] = 1
    a[i][9] = 1

print(a)