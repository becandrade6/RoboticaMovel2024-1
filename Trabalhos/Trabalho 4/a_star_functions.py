import numpy as np
import heapq
import math
import matplotlib.pyplot as plt

def distancia(ponto1, ponto2):
    return np.sqrt((ponto1[0] - ponto2[0])**2 + (ponto1[1] - ponto2[1])**2)

def ajusta_angulo(angulo):
    angulo = angulo % (2 * math.pi)
    if angulo > math.pi:
        angulo  = angulo - 2 * math.pi
    return angulo

def simula_processo(objetivos,ganhos_controlador,posicaoInicial,
                    toleranciaDistancia = 0.1, vmax = 0.5, 
                    wmax = np.deg2rad(180), dt = 0.1):
    
    Kp, Ki, Kd = ganhos_controlador

    x0,y0 = posicaoInicial
    xg, yg = objetivos[0]
    ex=[x0,y0,0.0]
    posicao = np.array(ex,float)
    historico_posicao = [(x0,y0,0.0)]
    x, y = x0,y0
    theta = 0

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

    for objetivo in objetivos:
        xg, yg = objetivo
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

            v = vmax
            w = Kp * alpha + Ki * integral_alpha + Kd * derivada_alpha
            w = np.sign(w) * min(abs(w), wmax)

            variacaoVelocidade = [v*math.cos(theta), v*math.sin(theta), w]
            deslocamento = np.array(variacaoVelocidade) * dt
            posicao = np.array(posicao) + deslocamento
            posicao[2] = ajusta_angulo(posicao[2])
            historico_posicao.append(posicao)
            x, y, theta = posicao

    return  historico_posicao

def a_estrela(mapa, inicio, objetivo,conectividade):
    linhas, colunas = mapa.shape
    lista_aberta = []
    heapq.heappush(lista_aberta, (0, inicio))
    custo_g = {inicio: 0}
    custo_f = {inicio: distancia(inicio, objetivo)}
    veio_de = {}

    while lista_aberta:
        _, atual = heapq.heappop(lista_aberta)

        if atual == objetivo:
            caminho = []
            while atual in veio_de:
                caminho.append(atual)
                atual = veio_de[atual]
            caminho.append(inicio)
            return caminho[::-1], custo_g[objetivo]  # Caminho do início ao objetivo

        for direcao in conectividade:
            vizinho = (atual[0] + direcao[0], atual[1] + direcao[1])
            if 1 <= vizinho[0] < linhas - 1 and 1 <= vizinho[1] < colunas - 1:
                if mapa[vizinho] == -1:
                    continue
                tentativa_g = custo_g[atual] + distancia(atual, vizinho)
                
                if vizinho not in custo_g or tentativa_g < custo_g[vizinho]:
                    veio_de[vizinho] = atual
                    custo_g[vizinho] = tentativa_g
                    custo_f[vizinho] = tentativa_g + distancia(vizinho, objetivo)
                    heapq.heappush(lista_aberta, (custo_f[vizinho], vizinho))
    
    return [], float('inf')  # Se nenhum caminho for encontrado

def gerar_mapa(tamanho, obstaculos=[]):
    mapa = np.zeros((tamanho + 2, tamanho + 2))
  
    # Adiciona obstáculos personalizados
    for obstaculo in obstaculos:
        x, y = obstaculo
        mapa[x, y] = -1
        
    # Adiciona a margem
    mapa[0, :] = -1
    mapa[-1, :] = -1
    mapa[:, 0] = -1
    mapa[:, -1] = -1

    return mapa


def plotar_mapa(mapa, inicio, objetivo, trajeto, caminho=None, custo_caminho=None, nomeFigura='mapa.png'):
    fig, ax = plt.subplots(figsize=(20, 20))
    
    # Destaca a margem com cores diferentes
    mapa_com_margem = np.copy(mapa)
    mapa_com_margem[0, :] = 0.5
    mapa_com_margem[-1, :] = 0.5
    mapa_com_margem[:, 0] = 0.5
    mapa_com_margem[:, -1] = 0.5
    
    #Plota mapa de acordo com gradiente de cor escolhido, no caso o "cividis"
    ax.imshow(mapa_com_margem, cmap='cividis', origin='lower')

    # Marcadores maiores
    ax.plot(inicio[1], inicio[0], 'go', markersize=20)  # Ponto inicial verde
    ax.plot(objetivo[1], objetivo[0], 'ko', markersize=20)  # Ponto objetivo preto

    if caminho:
        x = [ponto[1] for ponto in caminho]
        y = [ponto[0] for ponto in caminho]
        #Plota caminho achado entre os pontos
        #ax.plot(x, y, linewidth=3)  
        for x, y in caminho:
            ax.plot(y, x, 'b.', markersize=15)  

    for i in range(mapa.shape[0]):
        for j in range(mapa.shape[1]):
            if mapa[i, j] == -1:
                ax.text(j, i, '*', ha='center', va='center', color='red', fontsize=15)

    # Configurando ticks e grid
    tamanho = mapa.shape[0] - 2
    ax.set_xticks(np.arange(0, tamanho + 2, 1))
    ax.set_yticks(np.arange(0, tamanho + 2, 1))
    ax.set_xticks(np.arange(-0.5, tamanho + 1.5, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, tamanho + 1.5, 1), minor=True)
    ax.grid(which='minor', color='k', linestyle='-', linewidth=2)

    # Configurando rótulos dos ticks para serem de 1 a tamanho
    ax.set_xticklabels(np.arange(0, tamanho + 2, 1), fontsize=18)
    ax.set_yticklabels(np.arange(0, tamanho + 2, 1), fontsize=18)

    ax.set_xlabel('X', fontsize=24)
    ax.set_ylabel('Y', fontsize=24)
    ax.set_aspect('equal')

    if custo_caminho:
        ax.set_title(f'Custo total do caminho: {custo_caminho:.4f}', fontsize=28, fontweight='bold')
    else:
        ax.set_title('Nenhum caminho encontrado', fontsize=28, fontweight='bold')

    if trajeto is not None:
        x_trajeto = [ponto[1] for ponto in trajeto]
        y_trajeto = [ponto[0] for ponto in trajeto]
        theta_trajeto = [ponto[2] for ponto in trajeto]

        ax.plot(x_trajeto, y_trajeto, color='green', linewidth=2)

        ax.plot(x_trajeto[0], y_trajeto[0], 'ko', markersize=15, fillstyle='none')
        ax.plot(x_trajeto[-1], y_trajeto[-1], 'ro', markersize=15, fillstyle='none')

    # Para transformar em um mapa em centímetros onde cada quadrado tem 100cm (meio do quadrado tem 50cm)
    # Descomente as linhas abaixo:
    # cm_ticks = np.arange(-50, (tamanho + 2) * 100 - 50, 100)
    # ax.set_xticks(np.arange(0, tamanho + 2, 1))
    # ax.set_yticks(np.arange(0, tamanho + 2, 1))
    # ax.set_xticklabels(cm_ticks, fontsize=13)
    # ax.set_yticklabels(cm_ticks, fontsize=13)
    # ax.set_xlabel('X (cm)', fontsize=24)
    # ax.set_ylabel('Y (cm)', fontsize=24)

    plt.savefig(nomeFigura)

