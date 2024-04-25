import math
import csv
import numpy as np
import matplotlib.pyplot as plt

#ler os valores do arquivo t100a01.csv na pasta Leituras
numeroPulsosPorVolta = 900
diametroRodaEmMetros = 0.042

#calcule a distancia percorrida pela roda por leitura do pulso e armazene na variavel dS
dS = (diametroRodaEmMetros * math.pi) / numeroPulsosPorVolta

# Ler o arquivo t100a01_convertido.csv na pasta Leituras
with open('Leituras/t100a01_convertido.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  
    u = np.array([])
    y = np.array([])
    for row in reader:
        u = np.append(u, int(row[0])*dS/0.1) #m/s
        y = np.append(y, int(row[1])*dS/0.1) #m/s
N = len(u)

#crie a matriz Y
Y = np.reshape(y[1:], (-1, 1))
#crie a matriz X
X = np.vstack((y[:-1], u[1:])).T

#calcule o valor de theta
theta = np.linalg.inv(X.T @ X) @ X.T @ Y
#valores esperados
#a = 0,32
#b = 0,00....
print('a = ', theta[0][0], 'b = ', theta[1][0])

#calcula o valor de yh a partir dos thetas obtidos
yh = np.zeros(N)
yh[0] = 0
for i in range(1, N):
    yh[i] = theta[0][0] * yh[i-1] + theta[1][0] * u[i-1]

plt.plot(y, label='y')
plt.plot(yh, label='yh')
plt.legend()
plt.savefig('identificaSistemas.png')

mse = np.sum((yh-y)**2)/N
print("Erro médio quadrático:", mse)