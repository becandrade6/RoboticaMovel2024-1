from a_star_functions import *

# Definições das direções possíveis de movimento (4-direções ou 8-direções)
CONNECTIVITY_4 = [(0, 1), (1, 0), (0, -1), (-1, 0)]
CONNECTIVITY_8 = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]

# Tamanho do mapa
tamanho = 30

# Definir pontos inicial e objetivo
inicio = (7,4)
objetivo = (9,9)

# Definir obstáculos personalizados (opcional)
#obstaculos_personalizados = [(4,1),(9,1),(6,2),(9,2),(1,4),(4,4),(5,4),(6,4),(7,5),(3,6),(7,6),(3,7),(7,7),(9,5),(8,5),(10,5),(6,5)]

# Obstáculo não convexo
obstaculos_personalizados = [(3,5),(4,3),(4,6),(1,4),(4,4),(4,5),(5,6),(7,6),(7,7),(8,3),(8,4),(8,5),(8,6),(10,5),(6,6),(9,3),(9,7)]

# Obstáculo impossível
#obstaculos_personalizados = [(5,1),(5,2),(5,3),(5,4),(5,5),(5,6),(5,7),(5,8),(5,9),(5,10)]

# Obstáculos 30 X 30
#obstaculos_personalizados = [(15,10),(15,11),(15,12),(15,13),(15,14),(15,15),(15,16),(15,17),(15,18),(15,19),(15,20)]


# Gerar mapa
mapa_inicial = gerar_mapa(tamanho, obstaculos_personalizados)

# Executar o algoritmo A*
caminho,custo_caminho = a_estrela(mapa_inicial, inicio, objetivo,CONNECTIVITY_4)

# Plotar mapa final com o caminho encontrado
if caminho:
    # Gerar trajeto com o controlador
    controlador = [10,0.5,3]
    trajeto = simula_processo(caminho,controlador,caminho[0])

    #Plotar mapa inicial
    #plotar_mapa(mapa_inicial, inicio, objetivo ,trajeto=None, nomeFigura='mapainicial.png')

    mapa_final = mapa_inicial.copy()
    for passo in caminho:
        mapa_final[passo] = 3
    plotar_mapa(mapa_final, inicio, objetivo, trajeto, caminho, custo_caminho, nomeFigura= 'testeFinal30x30.png')
else:
    print("\nNenhum caminho encontrado.")
    plotar_mapa(mapa_inicial, inicio, objetivo,trajeto=None, nomeFigura='30x30Teste.png')
