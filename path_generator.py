import numpy as np
import matplotlib.pyplot as plt

n = 5
path1 = np.loadtxt('path1.csv', delimiter=',') / n
path2 = np.loadtxt('path2.csv', delimiter=',') / n
path3 = np.loadtxt('path3.csv', delimiter=',') / n
path4 = np.loadtxt('path4.csv', delimiter=',') / n
path5 = np.loadtxt('path5.csv', delimiter=',') / n
path6 = np.loadtxt('path6.csv', delimiter=',') / n
path7 = np.loadtxt('path7.csv', delimiter=',') / n
path8 = np.loadtxt('path8.csv', delimiter=',') / n
path9 = np.loadtxt('path9.csv', delimiter=',') / n
obstacles = np.loadtxt('obstacles.csv', delimiter=',') / n


path = np.concatenate((path1, path2, path3, path4, path5, path6, path7, path8, path9))
print(path.shape)

plt.scatter(path1[:, 0], path1[:, 1], color='r')
plt.scatter(path2[:, 0], path2[:, 1], color='b')
plt.scatter(path3[:, 0], path3[:, 1], color='g')
plt.scatter(path4[:, 0], path4[:, 1], color='c')
plt.scatter(path5[:, 0], path5[:, 1], color='m')
plt.scatter(path6[:, 0], path6[:, 1], color='y')
plt.scatter(path7[:, 0], path7[:, 1], color=(0.2, 0.6, 0.9))
plt.scatter(path8[:, 0], path8[:, 1], color=(0.9, 0.6, 0.9))
plt.scatter(path9[:, 0], path9[:, 1], color=(0.6, 0.9, 0.0))
plt.scatter(obstacles[:, 0], obstacles[:, 1], color='k')
plt.show()

np.save('path', path)