from twiddleFunctions import twiddlePID, plotar_trajeto_e_erro
import numpy as np

objetivo1 = np.array([3.1, 3.8,np.deg2rad(90)],float)
erroObjetivo1, parametros1, historicoErros1, melhor_trajeto1 = twiddlePID(objetivo1)
plotar_trajeto_e_erro(melhor_trajeto1, historicoErros1, objetivo1,'imagens/imagemFinalTeste1.png')
