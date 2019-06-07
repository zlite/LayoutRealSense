import numpy as np
import matplotlib.pyplot as plt

def plot_points(matrix, ls='--', lw=1.2, colors=None):
  x_points, y_points = matrix
  print(x_points)
  size = len(x_points[0])
  print("Size", size)
  colors = ['red', 'blue', 'orange', 'green'] if not None else colors
  for i in range(size):
#    print("Color", i, "=",colors[i])
    plt.plot(x_points[0][i], y_points[0][i], color=colors[i], marker='o')
    plt.plot([x_points[0][i], x_points[0][(i+1) % size]],
             [y_points[0][i], y_points[0][(i+1) % size]],color=colors[i], linestyle='--', linewidth=1.2)

A = np.matrix([[1, 0, -0.75*20], [0, 1, -0.25*20], [0, 0, 1]]) 
P = np.matrix([[0, 0, 20, 20], [0, 20, 20, 0], [1, 1, 1, 1]])
x_points = P[0]
y_points = P[1]
matrix = np.array([x_points, y_points])
colors = ['red', 'blue', 'orange', 'green']
plt.ylim([-25,25])
plt.xlim([-25,25])
plt.axes().set_aspect('equal')
plot_points(matrix, colors)

translated_matrix = A*P
#print (translated_matrix)
x_points = translated_matrix[0]
y_points = translated_matrix[1]
#temp = np.concatenate(x_points)
#print("X:", temp)
#print("Y:", y_points)
matrix = np.array([x_points, y_points])
plot_points(matrix, colors)
plt.show()


##x_points = np.array([0, 0, 20, 20])
##y_points = np.array([0, 20, 20, 0])
##matrix = np.array([x_points, y_points])
##colors = ['red', 'blue', 'orange', 'green']
##size = len(x_points)
##plot_points(matrix, colors)
##plt.ylim([-5,25])
##plt.xlim([-5,25])
##plt.axes().set_aspect('equal')
##plt.show()


ax = plt.axes()
ax.arrow(0, 0, 1, 0, head_width=0.05, head_length=0.1, fc='k', ec='k', zorder=20)
ax.arrow(0, 0, 0, 1, head_width=0.05, head_length=0.1, fc='k', ec='k', zorder=20)
ax.arrow(0, 0, np.cos(np.deg2rad(-30)), np.sin(np.deg2rad(-30)), head_width=0.05, head_length=0.1, fc='r', ec='r', zorder=10)
ax.arrow(0, 0, -np.sin(np.deg2rad(-30)), np.cos(np.deg2rad(-30)), head_width=0.05, head_length=0.1, fc='r', ec='r', zorder=10)
 
ax.spines['left'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['bottom'].set_position('zero')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.yaxis.set_ticks_position('left')
ax.set_aspect('equal')
plt.ylim([-0.7,1.2])
plt.xlim([-0.1,1.2])
plt.show()

ax = plt.axes()
ax.arrow(0, 0, 1, 0, head_width=0.05, head_length=0.1, fc='k', ec='k', zorder=20)
ax.arrow(0, 0, 0, 1, head_width=0.05, head_length=0.1, fc='k', ec='k', zorder=20)
ax.arrow(0, 0, 2, 0, head_width=0.05, head_length=0.1, fc='r', ec='r', zorder=10)
ax.arrow(0, 0, 0, 2, head_width=0.05, head_length=0.1, fc='r', ec='r', zorder=10)
 
ax.spines['left'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['bottom'].set_position('zero')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.yaxis.set_ticks_position('left')
ax.set_aspect('equal')
plt.ylim([-0.1,2.2])
plt.xlim([-0.1,2.2])
plt.show()



