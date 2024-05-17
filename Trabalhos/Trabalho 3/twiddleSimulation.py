from twiddleFunctions import twiddlePID, twiddle_para_multiplos_objetivos, plotar_trajeto
import numpy as np

objetivo = [35,28,np.deg2rad(90)]

erroObjetivo, parametros, historicoErros, melhor_trajeto = twiddlePID(objetivo)

plotar_trajeto(melhor_trajeto,"testeTwiddleFinal.png")