import os
import numpy as np
import matplotlib.pyplot as plt

fig,ax = plt.subplots()
file = np.loadtxt("before_filter.txt")
time = np.loadtxt("before_filter_time.txt")
file = np.array(file)
time = np.array(time)
ax.plot(time,file,color='red',linewidth=1,)


file = np.loadtxt("test.txt")
time = np.loadtxt("testtime.txt")
file = np.array(file)
time = np.array(time)

ax.plot(time,file)

plt.show()