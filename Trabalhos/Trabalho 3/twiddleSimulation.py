from twiddleFunctions import twiddlePID, twiddle_para_multiplos_objetivos, plotar_trajeto
import numpy as np

objetivo = [-20,-10,np.deg2rad(45)]

erroObjetivo, parametros, historicoErros, historicoErros_simulacao, historicoTrajetos = twiddlePID(objetivo)

plotar_trajeto(historicoTrajetos[-1],"testeTwiddleFinal.png")