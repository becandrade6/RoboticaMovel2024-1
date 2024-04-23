# Simulação aula 08 Modelo cinemático diferencial

## Sem perfumaria
- Pega posição inicial
- seta raio da roda e metade do comprimento do eixo das rodas
- seta velocidades angulares das duas rodas
- calcula velocidade linear do ponto-médio
- calcula velocidade angular do ponto-médio
- armazena posição inicial antes de começar a simulação
- define tempo de amostragem da simulação
- define tempo máximo da simulação
- começa for da simulação (for t = 0:dt:tmax)
    - pega a posição angular theta do robo atual (a ultima posição conhecida)
    - calcula as velocidades linear e angular do ponto médio (em programas com algoritmos de controle eles entram aqui pq mudam a cada instante de tempo t)
    - calcula a variação da velocidade no instante de tempo t atual, usando as velocidades calculadas acima
    - atualiza a posição atual, somando a variação na posição atual. P = P + dP * dt
    - armazena as posições do robô ao longo do tempo (histórico)
    - plota os resultados da iteração

## Com perfumaria
- Define valores para "sólidos geométricos" que representarão o robô diferencial bidimensional
- Define valores para rodas
- executa o algoritmo sem perfumaria
- no final, após armazenar as posições do robo ao longo do tempo, devemos executar as transições de frame para atualizar os valores da perfumaria
- plotamos os sólidos com os valores atualizados a cada iteração

### Funções de translações utilizadas
- Translação bidimensional
- Rotação no eixo-z bidimensional