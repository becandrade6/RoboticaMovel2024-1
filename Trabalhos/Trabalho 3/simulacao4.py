from twiddleFunctions import twiddlePID,plotar_trajeto_e_erro
import numpy as np

objetivo4 = np.array([-3.8,-3.1,np.deg2rad(90)],float)
erroObjetivo4, parametros4, historicoErros4, melhor_trajeto4 = twiddlePID(objetivo4)
plotar_trajeto_e_erro(melhor_trajeto4, historicoErros4, objetivo4,'imagens/imagemFinalTeste4.png')