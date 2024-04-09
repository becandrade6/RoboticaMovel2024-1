import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
from funcoesTrab1 import *

t = sp.symbols('t')

v = sp.Matrix([-20*sp.sin(t), 20*(sp.cos(t)**2 - sp.sin(t)**2)])

p = sp.integrate(v, t)
#resultado = Matrix([[20*cos(t)], [20*sin(t)*cos(t)]])
a = sp.diff(v, t)
# resultado = Matrix([[-20*cos(t)], [-80*sin(t)*cos(t)]])
p0 = sp.Matrix([20, 0])

c1 = sp.symbols('c1')
pxSimbolica = p[0].subs(t,0) + c1
#Resultado = 20 + c1 -> resolvendo 20 + C1 = 20, C1 = 0
c2 = sp.symbols('c2')
pySimbolica = p[1].subs(t,0) + c2
#Resultado = 0 + c2 -> resolvendo 0 + C2 = 0, C2 = 0

#------------------------------------------------
#QUESTAO 2
timeVector = np.linspace(0, 2*np.pi, int(2*np.pi/(np.pi/150)))
p0_func = sp.lambdify(t, p[0], "numpy")
p1_func = sp.lambdify(t, p[1], "numpy")
v0_func = sp.lambdify(t, v[0], "numpy")
v1_func = sp.lambdify(t, v[1], "numpy")
a0_func = sp.lambdify(t, a[0], "numpy")
a1_func = sp.lambdify(t, a[1], "numpy")

px = p0_func(timeVector)
py = p1_func(timeVector)

vx = v0_func(timeVector)
vy = v1_func(timeVector)

aX = a0_func(timeVector)
aY = a1_func(timeVector)

plots = [[px, py], [vx, vy], [aX, aY]]
titles = ['Posição', 'Velocidade', 'Aceleração']

for index, plot in enumerate(plots):
    fig, ax = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle('Gráfico de '+titles[index],fontweight='bold')
    ax[0].plot(timeVector, plot[1],label='Real')
    ax[0].set_xlabel('Tempo (s)')
    ax[0].set_ylabel(titles[index] + ' em y')

    ax[1].plot(timeVector, plot[0])
    ax[1].set_xlabel('Tempo (s)')
    ax[1].set_ylabel(titles[index] + ' em x')

    ax[2].plot(plot[0], plot[1])
    ax[2].set_xlabel(titles[index] + ' em x')
    ax[2].set_ylabel(titles[index] + ' em y')

    handles,labels = ax[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right')
    fig.savefig('/home/levty/Documentos/UFJF/9_Periodo/Robotica Movel/Trabalho 1/questao2_'+titles[index]+'.png')    
#------------------------------------------------
#QUESTAO 3
passosAmostrais = [np.pi/10,np.pi/100,np.pi/300]
vetoresPorPassoPosicao = {}
vetoresPorPassoVelocidade = {}
vetoresPorPassoAceleracao = {}
# {
#   np.pi/10: [vetorX, vetorY, vetorTempo],
#   np.pi/100: [vetorX, vetorY, vetorTempo],
#   np.pi/300: [vetorX, vetorY, vetorTempo]
#}
for passo in passosAmostrais:
    vetoresPorPassoPosicao[passo] = [integracaoNewton(velocidadeX, [20], 0, 2*np.pi, passo), integracaoNewton(velocidadeY, [0], 0, 2*np.pi, passo),np.linspace(0, 2*np.pi, int((2*np.pi)/passo)+1)]
    vetoresPorPassoVelocidade[passo] = [velocidadeX(np.linspace(0, 2*np.pi, int((2*np.pi)/passo)+1)), velocidadeY(np.linspace(0, 2*np.pi, int((2*np.pi)/passo)+1)),np.linspace(0, 2*np.pi, int((2*np.pi)/passo)+1)]
    vetoresPorPassoAceleracao[passo] = [derivacaoNewton(velocidadeX, 0, 2*np.pi, passo), derivacaoNewton(velocidadeY, 0, 2*np.pi, passo),np.linspace(0, 2*np.pi, int((2*np.pi)/passo))]

simulacao = {
    'Posição': [vetoresPorPassoPosicao,px,py],
    'Velocidade': [vetoresPorPassoVelocidade,vx,vy],
    'Aceleracao': [vetoresPorPassoAceleracao,aX,aY]
}
# {
#  Velocidade: vetoresPorPassoVelocidade
#  Posicao: vetoresPorPassoPosicao
#  Aceleracao: vetoresPorPassoAceleracao
#}
    #vetoresPorPassoVelocidade = {
    #   np.pi/10: [vetorX, vetorY, vetorTempo],
    #   np.pi/100: [vetorX, vetorY, vetorTempo],
    #   np.pi/300: [vetorX, vetorY, vetorTempo]
    #}
    #...
for grandeza, simulacaoArray in simulacao.items():
    for key, value in simulacaoArray[0].items():
        fig, ax = plt.subplots(3, 1, figsize=(10, 10))
        fig.suptitle('Simulação de '+grandeza,fontweight='bold')
        ax[0].plot(value[2], value[1], color='r',label='Simulado com passo '+str(key))
        ax[0].plot(timeVector, simulacaoArray[2], color='b', label='Real')
        ax[0].set_xlabel('Tempo (s)')
        ax[0].set_ylabel(grandeza+ ' em y')
        ax[0].grid(True)

        ax[1].plot(value[2], value[0],color='r', label='Simulado com passo '+str(key))
        ax[1].plot(timeVector, simulacaoArray[1], color='b', label='Real')
        ax[1].set_xlabel('Tempo (s)')
        ax[1].set_ylabel(grandeza+ ' em x')
        ax[1].grid(True)

        ax[2].plot(value[0], value[1],color='r', label='Simulado com passo '+str(key))
        ax[2].plot(simulacaoArray[1], simulacaoArray[2], color='b', label='Real')
        ax[2].set_xlabel(grandeza+ ' em x')
        ax[2].set_ylabel(grandeza+ ' em y')
        ax[2].grid(True)

        handles,labels = ax[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right')
        fig.savefig('/home/levty/Documentos/UFJF/9_Periodo/Robotica Movel/Trabalho 1/questao3_'+grandeza+'_passo_'+str(key)+'.png')