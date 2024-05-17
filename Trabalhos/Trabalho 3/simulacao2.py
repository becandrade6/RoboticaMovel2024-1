from twiddleFunctions import twiddlePID,plotar_trajeto_e_erro
import numpy as np
objetivo2 = np.array([3.1,-3.8,np.deg2rad(90)],float)
erroObjetivo2, parametros2, historicoErros2, melhor_trajeto2 = twiddlePID(objetivo2)
plotar_trajeto_e_erro(melhor_trajeto2, historicoErros2, objetivo2,'imagens/imagemFinalTeste2.png')
