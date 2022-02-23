import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Cartesian axes
#ax.quiver(-1, 0, 0, 3, 0, 0, color='#aaaaaa', linestyle='dashed')
#ax.quiver(0, -1, 0, 0, 3, 0, color='#aaaaaa', linestyle='dashed')
#ax.quiver(0, 0, -1, 0, 0, 3, color='#aaaaaa', linestyle='dashed')
# Vector before rotation
#ax.quiver(0, 0, 0, 1, 0, 0, color='b')
# Vector after rotation
#ax.quiver(0, 0, 0, 0, 0.71, -0.71, color='r')

x = 1
y = 0
z = 1



x_angle = 6.09 * 10**-3
y_angle = -1
z_angle = 6.09 * 10**-3

ax.quiver(x, y, z, x_angle, y_angle, z_angle, color='b')

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
plt.show()