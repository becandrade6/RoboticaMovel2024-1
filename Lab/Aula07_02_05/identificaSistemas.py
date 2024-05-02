import math
import csv
import numpy as np
import matplotlib.pyplot as plt

#ler os valores do arquivo t100a01.csv na pasta Leituras
numeroPulsosPorVolta = 900
diametroRodaEmMetros = 0.042

#deltaV = deltaS/deltaT -> deltaS = deltaV*deltaT
#calcule a distancia percorrida pela roda por leitura do pulso e armazene na variavel dS
dS = (diametroRodaEmMetros * math.pi) / numeroPulsosPorVolta
arquivos = ['t100a01', 't100a03', 't100a05', 't100a07']
arquivosTeste = ['t100a02', 't100a04', 't100a06', 't100a08']
# Ler o arquivo t100a01_convertido.csv na pasta Leituras
for arquivo in arquivos:
    with open('Leituras/'+arquivo+'_convertido.csv', 'r') as file:
        reader = csv.reader(file)
        next(reader)  
        u = np.array([])
        y = np.array([])
        for row in reader:
            u = np.append(u, int(row[0])) #PWM
            y = np.append(y, int(row[1])*dS/0.1) #m/s
    N = len(u)

    #crie a matriz Y
    Y = np.reshape(y[1:], (-1, 1))
    #crie a matriz X
    X = np.vstack((y[:-1], u[:-1])).T

    #calcule o valor de theta
    theta = np.linalg.inv(X.T @ X) @ X.T @ Y
    #valores esperados
    #a = 0,32
    #b = 0,00....
    print('Thetas para ',arquivo,' a = ', theta[0][0], 'b = ', theta[1][0])

    #calcula o valor de yh a partir dos thetas obtidos
    yh = np.zeros(N)
    yh[0] = 0
    for i in range(1, N):
        yh[i] = theta[0][0] * yh[i-1] + theta[1][0] * u[i-1]

    plt.plot(y, label='y')
    plt.plot(yh, label='yh')
    plt.legend()
    plt.savefig('identificaSistemas'+'_'+arquivo+'.png')
    plt.close()

    mse = np.sum((yh-y)**2)/N
    print("Erro médio quadrático:", mse)

    #testa com outro arquivo de valores tb coletados
    indiceArquivo = arquivos.index(arquivo)
    arquivoTeste = arquivosTeste[indiceArquivo]
    with open('Leituras/'+arquivoTeste+'_convertido.csv', 'r') as file:
        reader = csv.reader(file)
        next(reader)  
        u = np.array([])
        y = np.array([])
        for row in reader:
            y = np.append(y, int(row[1])*dS/0.1) #m/s
            u = np.append(u, int(row[0]))
    N = len(u)
    yhTeste = np.zeros(N)
    yhTeste[0] = 0
    for i in range(1, N):
        yhTeste[i] = theta[0][0] * y[i-1] + theta[1][0] * u[i-1]
    plt.plot(y, label='y')
    plt.plot(yhTeste, label='yh')
    plt.legend()
    plt.savefig('identificaSistemasValidacao'+'_'+arquivoTeste+'.png')
    plt.close()
    mseTeste = np.sum((yhTeste-y)**2)/N
    print("Erro médio quadrático teste validação para ",arquivoTeste,":", mseTeste)

    