import math
import numpy as np
import matplotlib.pyplot as plt

def ajusta_angulo(angulo):
    angulo = angulo % (2 * math.pi)
    if angulo > math.pi:
        angulo  = angulo - 2 * math.pi
    return angulo

def simula_processo(objetivo,ganhos_controlador,posicaoInicial = np.array([0.0,0.0,0.0],float),
                    toleranciaDistancia = 0.001, vmax = 0.5, 
                    wmax = np.deg2rad(180), dt = 0.1):
    
    historico_posicao = [posicaoInicial]
    historico_erro = np.array([],float)
    Kp, Ki, Kd = ganhos_controlador

    posicao = np.array(posicaoInicial,float)

    #calculando parametros da reta entre o ponto de partida e objetivo
    x0, y0, theta0 = posicaoInicial
    xg, yg, thetag = objetivo
    x, y, theta = posicao

    a = yg - y0
    b = x0 - xg
    c = xg*y0 - x0*yg

    erro = abs(a*x + b*y + c)/math.sqrt(a**2 + b**2)
    historico_erro = np.append(historico_erro,erro)

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
        historico_erro = np.append(historico_erro,erro)
    return historico_erro.sum(), historico_erro, historico_posicao

def twiddlePID(objetivo, posicaoInicial = [0,0,0],
               ganhos_iniciais_controlador = np.array([0.0,0.0,0.0],float), 
               diferenciais_parametros = np.array([1.,1.,1.],float),
               ksi = 0.01, delta = 0.001):
    parametros = ganhos_iniciais_controlador
    melhor_erro, _, _ = simula_processo(objetivo,ganhos_iniciais_controlador)
    melhores_parametros = parametros
    contador_iteracoes = 0
    while diferenciais_parametros.sum() > delta:
        contador_iteracoes += 1

        for i in range(len(parametros)):
            parametros[i] += diferenciais_parametros[i]
            erro, _, _ = simula_processo(objetivo,parametros)
            if erro < melhor_erro:
                melhor_erro = erro
                melhores_parametros = parametros
                diferenciais_parametros[i] +=  diferenciais_parametros[i]*ksi
            else:
                parametros[i] -= 2 * diferenciais_parametros[i]
                erro, _, _ = simula_processo(objetivo,parametros)
                if erro < melhor_erro:
                    melhor_erro = erro
                    melhores_parametros = parametros
                    diferenciais_parametros[i] +=  diferenciais_parametros[i]*ksi
                else:
                    parametros[i] += diferenciais_parametros[i]
                    diferenciais_parametros[i] -=  diferenciais_parametros[i]*ksi    
        print("Rodada:", contador_iteracoes)
        print("Melhor erro:", round(melhor_erro, 4))
        print("Soma dos diferenciais:", round(sum(diferenciais_parametros), 6))
    print("Parâmetros: P =  %.4f , I =  %.4f , D = %.4f" % (melhores_parametros[0], melhores_parametros[1], melhores_parametros[2]))
    #simular com os melhores parametros para obter novamente o melhor trajeto
    melhor_erro, historico_erros, melhor_trajeto = simula_processo(objetivo,melhores_parametros)
    return melhor_erro, melhores_parametros, historico_erros, melhor_trajeto

def plotar_trajeto(trajeto, caminho_imagem):
    x_trajeto = [ponto[0] for ponto in trajeto]
    y_trajeto = [ponto[1] for ponto in trajeto]
    theta_trajeto = [ponto[2] for ponto in trajeto]
    
    fig, ax = plt.subplots()
    ax.plot(x_trajeto, y_trajeto, 'b-', linewidth=1)
    
    ax.plot(x_trajeto[0], y_trajeto[0], 'ko', markersize=8, fillstyle='none')
    ax.plot(x_trajeto[-1], y_trajeto[-1], 'ro', markersize=8, fillstyle='none')
    
    comprimento_barra = 2
    x_barra_inicial = x_trajeto[0] + comprimento_barra * np.cos(theta_trajeto[0])
    y_barra_inicial = y_trajeto[0] + comprimento_barra * np.sin(theta_trajeto[0])
    ax.plot([x_trajeto[0], x_barra_inicial], [y_trajeto[0], y_barra_inicial], 'k-', linewidth=2)
    
    print(np.rad2deg(theta_trajeto[-1]))
    x_barra_final = x_trajeto[-1] + comprimento_barra * np.cos(theta_trajeto[-1])
    y_barra_final = y_trajeto[-1] + comprimento_barra * np.sin(theta_trajeto[-1])
    ax.plot([x_trajeto[-1], x_barra_final], [y_trajeto[-1], y_barra_final], 'r-', linewidth=2)
    
    ax.set_xlim(min(x_trajeto) - 10, max(x_trajeto) + 10)
    ax.set_ylim(min(y_trajeto) - 10, max(y_trajeto) + 10)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)
    
    plt.savefig(caminho_imagem)

def plotar_trajeto_e_erro(trajeto,erro,objetivo,caminho_imagem,pontoInicial = np.array([0,0,0],float)):
    x_trajeto = [ponto[0] for ponto in trajeto]
    y_trajeto = [ponto[1] for ponto in trajeto]
    theta_trajeto = [ponto[2] for ponto in trajeto]

    x_objetivo, y_objetivo, theta_objetivo = objetivo
    x_inicial, y_inicial, theta_inicial = pontoInicial

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 10))

    ax1.plot(x_trajeto, y_trajeto, 'r-', linewidth=1)
    ax1.plot(x_inicial, y_inicial, 'ko', markersize=8, fillstyle='none')
    ax1.plot(x_objetivo, y_objetivo, 'ko', markersize=8, fillstyle='none')
    ax1.plot(x_trajeto[-1], y_trajeto[-1], 'ro', markersize=8, fillstyle='none')

    ax1.text(x_inicial + 0.02, y_inicial + 0.03, 'Start', fontsize=10,weight='bold')
    ax1.text(x_objetivo + 0.02, y_objetivo + 0.03, 'Goal', fontsize=10,weight='bold')
    
    comprimento_barra = 0.3
    x_barra_inicial = x_trajeto[0] + comprimento_barra * np.cos(theta_trajeto[0])
    y_barra_inicial = y_trajeto[0] + comprimento_barra * np.sin(theta_trajeto[0])
    ax1.plot([x_trajeto[0], x_barra_inicial], [y_trajeto[0], y_barra_inicial], 'k-', linewidth=2)
    
    x_barra_final = x_trajeto[-1] + comprimento_barra * np.cos(theta_trajeto[-1])
    y_barra_final = y_trajeto[-1] + comprimento_barra * np.sin(theta_trajeto[-1])
    ax1.plot([x_trajeto[-1], x_barra_final], [y_trajeto[-1], y_barra_final], 'r-', linewidth=2)

    ax1.plot([x_inicial, x_objetivo], [y_inicial, y_objetivo], 'g--', linewidth=1)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title('Navegação do robô', weight='bold')
    ax1.grid(True)

    # Plotar erro na metade direita
    historicoErrosMilimetros = erro*1000
    ax2.plot(historicoErrosMilimetros,'b-', linewidth=1)
    ax2.set_ylabel('Erro [mm]')
    ax2.set_xlabel('Tempo')
    ax2.set_title('Erros',weight='bold')
    ax2.grid(True)

    plt.savefig(caminho_imagem)
    plt.close()