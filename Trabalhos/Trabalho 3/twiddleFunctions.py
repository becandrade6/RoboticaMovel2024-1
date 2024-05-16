import math
import numpy as np
import matplotlib.pyplot as plt

def ajusta_angulo(angulo):
    angulo = angulo % (2 * math.pi)
    if angulo > math.pi:
        angulo  = angulo - 2 * math.pi
    return angulo

def simula_processo(objetivo,ganhos_controlador,posicaoInicial = [0,0,0],
                    toleranciaDistancia = 0.01, vmax = 0.25, 
                    wmax = np.deg2rad(180), dt = 0.1):
    
    historico_posicao = [posicaoInicial]
    historico_erro = []
    Kp, Ki, Kd = ganhos_controlador

    posicao = np.array(posicaoInicial)

    #calculando parametros da reta entre o ponto de partida e objetivo
    x0, y0, theta0 = posicaoInicial
    xg, yg, thetag = objetivo
    x, y, theta = posicao

    a = yg - y0
    b = x0 - xg

    c = xg*y0 - x0*yg

    erro = abs(a*x + b*y + c)/math.sqrt(a**2 + b**2)
    historico_erro.append(erro)

    integral_alpha = 0
    derivada_alpha = 0
    alpha_anterior = 0

    tempo_corrente = 0
    tempo_maximo = 10* math.sqrt((xg - x0)**2 + (yg - y0)**2)/vmax

    erroX = xg - x
    erroY = yg - y

    rho = math.sqrt(erroX**2 + erroY**2)
    gamma = ajusta_angulo(math.atan2(erroY,erroX))
    alpha = ajusta_angulo(gamma - theta)

    while rho > toleranciaDistancia and tempo_corrente <= tempo_maximo:
        tempo_corrente += dt
        erroX = xg - x
        erroY = yg - y

        rho = math.sqrt(erroX**2 + erroY**2)
        gamma = ajusta_angulo(math.atan2(erroY,erroX))
        alpha = ajusta_angulo(gamma - theta)
        
        derivada_alpha = (alpha - alpha_anterior)
        alpha_anterior = alpha
        integral_alpha += alpha

        v = min(rho, vmax)
        w = Kp * alpha + Ki * integral_alpha + Kd * derivada_alpha
        w = np.sign(w) * min(abs(w), wmax)

        variacaoVelocidade = [v*math.cos(theta), v*math.sin(theta), w]
        deslocamento = np.array(variacaoVelocidade) * dt
        posicao = np.array(posicao) + deslocamento
        posicao[2] = ajusta_angulo(posicao[2])
        historico_posicao.append(posicao)
        x, y, theta = posicao

        erro = abs(a*x + b*y + c)/math.sqrt(a**2 + b**2)
        historico_erro.append(erro)
    return sum(historico_erro), historico_erro, historico_posicao

def twiddlePID(objetivo, posicaoInicial = [0,0,0],
               ganhos_iniciais_controlador = np.array([0,0,0]), 
               diferenciais_parametros = np.array([1,1,1]),
               ksi = 0.01, delta = 0.001):
    parametros = ganhos_iniciais_controlador
    historico_erros = []
    historico_totalErros = []
    historico_totalTrajetos = []
    melhor_erro, historico_erroSimulado, trajeto_simulado = simula_processo(objetivo,ganhos_iniciais_controlador)
    historico_erros.append(melhor_erro)
    historico_totalErros.append(historico_erroSimulado)
    historico_totalTrajetos.append(trajeto_simulado)
    contador_iteracoes = 0
    while sum(diferenciais_parametros) > delta:
        contador_iteracoes += 1

        for i in range(len(parametros)):
            parametros[i] += diferenciais_parametros[i]
            
            erro, historico_erroSimulado, trajeto_simulado = simula_processo(objetivo,parametros)
            historico_erros.append(erro)
            historico_totalErros.append(historico_erroSimulado)
            historico_totalTrajetos.append(trajeto_simulado)
            if erro < melhor_erro:
                melhor_erro = erro
                diferenciais_parametros[i] *= (1 + ksi)
            else:
                parametros[i] -= 2 * diferenciais_parametros[i]
                erro, historico_erroSimulado, trajeto_simulado = simula_processo(objetivo,parametros)
                historico_erros.append(erro)
                historico_totalErros.append(historico_erroSimulado)
                historico_totalTrajetos.append(trajeto_simulado)
                if erro < melhor_erro:
                    melhor_erro = erro
                    diferenciais_parametros[i] *= (1 + ksi)
                else:
                    parametros[i] += diferenciais_parametros[i]
                    diferenciais_parametros[i] *= (1 - ksi)        
        print("Rodada:", contador_iteracoes)
        print("Melhor erro:", round(melhor_erro, 4))
        print("Soma dos diferenciais:", round(sum(diferenciais_parametros), 6))
    print("Parâmetros: P =  %.4f , I =  %.4f , D = %.4f" % (parametros[0], parametros[1], parametros[2]))
    return melhor_erro, parametros, historico_erros, historico_totalErros, historico_totalTrajetos

def twiddle_para_multiplos_objetivos(objetivos):
    melhores_trajetos = []
    objetivo_anterior = None
    for objetivo in objetivos:
        erroObjetivo, parametros, historicoErros, historicoErros_simulacao, historicoTrajetos = twiddlePID(objetivo, objetivo_anterior)
        melhores_trajetos.append(historicoTrajetos[-1])
        objetivo_anterior = objetivo
    # Extrair as coordenadas de cada melhor trajeto
    x_trajetos = [[ponto[0] for ponto in trajeto] for trajeto in melhores_trajetos]
    y_trajetos = [[ponto[1] for ponto in trajeto] for trajeto in melhores_trajetos]
    theta_trajetos = [[ponto[2] for ponto in trajeto] for trajeto in melhores_trajetos]
    # Plotar os trajetos
    fig, ax = plt.subplots()
    for i in range(len(objetivos)):
        ax.plot(x_trajetos[i], y_trajetos[i], linewidth=2)
        ax.plot(x_trajetos[i][0], y_trajetos[i][0], 'ko', markersize=8, fillstyle='none')
        ax.plot(x_trajetos[i][-1], y_trajetos[i][-1], 'ko', markersize=8)
        comprimento_barra = 0.5
        x_barra = x_trajetos[i][-1] + comprimento_barra * np.cos(theta_trajetos[i][-1])
        y_barra = y_trajetos[i][-1] + comprimento_barra * np.sin(theta_trajetos[i][-1])
        ax.plot([x_trajetos[i][-1], x_barra], [y_trajetos[i][-1], y_barra], 'r-', linewidth=2)
    # Configurar os limites do gráfico
    min_x = min([min(x_trajeto) for x_trajeto in x_trajetos]) - 1
    max_x = max([max(x_trajeto) for x_trajeto in x_trajetos]) + 1
    min_y = min([min(y_trajeto) for y_trajeto in y_trajetos]) - 1
    max_y = max([max(y_trajeto) for y_trajeto in y_trajetos]) + 1
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    # Configurar os rótulos dos eixos
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # Adicionar grade ao plot
    ax.grid(True)
    # Exibir o gráfico
    plt.savefig('trajetosFinais2.png')

def plotar_trajeto(trajeto, caminho_imagem):
    x_trajeto = [ponto[0] for ponto in trajeto]
    y_trajeto = [ponto[1] for ponto in trajeto]
    theta_trajeto = [ponto[2] for ponto in trajeto]
    
    fig, ax = plt.subplots()
    ax.plot(x_trajeto, y_trajeto, 'b-', linewidth=2)
    
    ax.plot(x_trajeto[0], y_trajeto[0], 'ko', markersize=8, fillstyle='none')
    ax.plot(x_trajeto[-1], y_trajeto[-1], 'ro', markersize=8, fillstyle='none')
    
    comprimento_barra = 2
    x_barra_inicial = x_trajeto[0] + comprimento_barra * np.cos(theta_trajeto[0])
    y_barra_inicial = y_trajeto[0] + comprimento_barra * np.sin(theta_trajeto[0])
    ax.plot([x_trajeto[0], x_barra_inicial], [y_trajeto[0], y_barra_inicial], 'k-', linewidth=2)
    
    x_barra_final = x_trajeto[-1] + comprimento_barra * np.cos(theta_trajeto[-1])
    y_barra_final = y_trajeto[-1] + comprimento_barra * np.sin(theta_trajeto[-1])
    ax.plot([x_trajeto[-1], x_barra_final], [y_trajeto[-1], y_barra_final], 'r-', linewidth=2)
    
    ax.set_xlim(min(x_trajeto) - 10, max(x_trajeto) + 10)
    ax.set_ylim(min(y_trajeto) - 10, max(y_trajeto) + 10)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)
    
    plt.savefig(caminho_imagem)

# objetivosTeste = [[20,15,np.deg2rad(45)], [10,10,np.deg2rad(90)], [30,30,np.deg2rad(135)]]
# twiddle_para_multiplos_objetivos(objetivosTeste)