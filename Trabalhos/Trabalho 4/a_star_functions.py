import numpy as np
import heapq
import matplotlib.pyplot as plt

def distancia(ponto1, ponto2):
    return np.sqrt((ponto1[0] - ponto2[0])**2 + (ponto1[1] - ponto2[1])**2)

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
    
    return None  # Se nenhum caminho for encontrado

def gerar_mapa(tamanho, obstaculos=[],start_index = 1):
    mapa = np.zeros((tamanho+1, tamanho+1))
    # Adiciona obstáculos personalizados
    for obstaculo in obstaculos:
        x, y = obstaculo
        mapa[x, y] = -1
    return mapa

def plotar_mapa(mapa, inicio, objetivo, caminho=None,custo_caminho=None, nomeFigura = 'mapa.png'):
    fig, ax = plt.subplots()
    ax.imshow(mapa, cmap='gray')

    ax.plot(inicio[1], inicio[0], 'go', markersize=10)  # Ponto inicial verde
    ax.plot(objetivo[1], objetivo[0], 'ko', markersize=10)  # Ponto objetivo preto

    if caminho:
        for x, y in caminho:
            ax.plot(y, x, 'b.', markersize=8)  # Caminho em azul

    for i in range(mapa.shape[0]):
        for j in range(mapa.shape[1]):
            if mapa[i, j] == -1:
                ax.text(j, i, '*', ha='center', va='center', color='red')  # Obstáculo como asterisco

    ax.set_xticks(np.arange(-0.5, mapa.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, mapa.shape[0], 1), minor=True)
    ax.grid(which='minor', color='k', linestyle='-', linewidth=2)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')

    if custo_caminho:
        ax.set_title(f'Custo total do caminho: {custo_caminho:.4f}')

    plt.savefig(nomeFigura)