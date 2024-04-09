import matplotlib.pyplot as plt

# Define the coordinates of the solid object
x = [-1, 1, 2, 1, -1, -1.5, -1.5]
y = [-1, -1, 0, 1, 1, 0.5, -0.5]

# Plot the solid object
plt.fill(x, y, 'b')

# Set the axis limits
plt.xlim(-2, 3)
plt.ylim(-2, 2)

# Display the plot
plt.savefig('/home/levty/Documentos/UFJF/9 Periodo/Robotica Movel/Aula 1 - Revisao Python/roboSolido.png')

