import matplotlib.pyplot as plt

# Define os pontos para a letra B
points = [(1.5,0),(1.5, 4.5), (2.5,4.5), (3.5,4), (3.25,3.25), (3,3), (2.5,2.73),(2,2.71),(2.5,2.5),(3,2.23),(3.25,1.86),(3.5,1.27),(3.24,0.75),(3,0.5),(2,0),(1.5,0)]
x_coords = [point[0] for point in points]
y_coords = [point[1] for point in points]

# Plota a letra B
plt.plot(x_coords, y_coords, 'r-')
plt.scatter(x_coords, y_coords, color='black')
plt.axis([0, 5, 0, 5])
# Adiciona números nos pontos
for i, point in enumerate(points):
    plt.text(point[0], point[1], str(i+1), fontsize=12, ha='center', va='bottom')
plt.savefig('letraB.png')
# Mostra o gráfico
# Mostra o gráfico com pontos pretos
