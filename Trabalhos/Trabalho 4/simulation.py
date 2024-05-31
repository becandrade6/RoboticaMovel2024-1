from a_star_functions import *

# Definições das direções possíveis de movimento (4-direções ou 8-direções)
CONNECTIVITY_4 = [(0, 1), (1, 0), (0, -1), (-1, 0)]
CONNECTIVITY_8 = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]

# Tamanho do mapa
tamanho = 10

# Definir pontos inicial e objetivo
inicio = (7,4)
objetivo = (9,9)

# Definir obstáculos personalizados (opcional)
obstaculos_personalizados = [(4,1),(9,1),(6,2),(9,2),(1,4),(4,4),(5,4),(6,4),(7,5),(3,6),(7,6),(3,7),(7,7),(9,5),(8,5),(10,5)]

# Gerar mapa
mapa_inicial = gerar_mapa(tamanho, obstaculos_personalizados)

# Executar o algoritmo A*
caminho,custo_caminho = a_estrela(mapa_inicial, inicio, objetivo,CONNECTIVITY_8)

# Plotar mapa inicial
plotar_mapa(mapa_inicial, inicio, objetivo, nomeFigura='mapainicial.png')

# Plotar mapa final com o caminho encontrado
if caminho:
    mapa_final = mapa_inicial.copy()
    for passo in caminho:
        mapa_final[passo] = 3
    plotar_mapa(mapa_final, inicio, objetivo, caminho,custo_caminho, nomeFigura= 'mapaFinal.png')
else:
    print("\nNenhum caminho encontrado.")
