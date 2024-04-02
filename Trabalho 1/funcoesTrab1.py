import numpy as np

def velocidadeX(t):
    return -20*np.sin(t)

def velocidadeY(t):
    return 20*(np.cos(t)**2 - np.sin(t)**2)

def integracaoNewton(func, estadoInicial, tempoInicial, tempoFinal, passoAmostral):
    result = estadoInicial
    tempos = np.linspace(tempoInicial, tempoFinal, int((tempoFinal - tempoInicial)/passoAmostral))
    for index in range(len(tempos)):
        result.append(result[index] + func(tempos[index])*passoAmostral)
    return result

def derivacaoNewton(func, tempoInicial, tempoFinal, passoAmostral):
    result = [float('Nan')]
    tempos = np.linspace(tempoInicial, tempoFinal, int((tempoFinal - tempoInicial)/passoAmostral))
    for index in range(len(tempos)):
        if index == 0:
            continue
        result.append((func(tempos[index]) - func(tempos[index-1]))/passoAmostral)
    return result