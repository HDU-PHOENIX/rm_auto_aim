import os
import numpy as np
import matplotlib.pyplot as plt

fig,ax = plt.subplots()
# file = np.loadtxt("before_filter.txt")
# time = np.loadtxt("before_filter_time.txt")
# file = np.array(file)
# time = np.array(time)
# ax.plot(time,file,'o',color='red',linewidth=1)


file = np.loadtxt("./record/omega.txt")
time = np.loadtxt("./record/omegatime.txt")
file = np.array(file)
time = np.array(time)

ax.scatter(time,file,s=2.)

plt.show()