from twiddleFunctions import twiddlePID, plotar_trajeto_e_erro
import numpy as np

objetivo3 = np.array([-3.8,3.1,np.deg2rad(90)],float)
erroObjetivo3, parametros3, historicoErros3, melhor_trajeto3 = twiddlePID(objetivo3)
plotar_trajeto_e_erro(melhor_trajeto3, historicoErros3, objetivo3,'imagens/imagemFinalTeste3.png')