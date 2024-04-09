% ==========================================
% Evoluindo a posição de um robô diferencial
% ==========================================
clear; clc; close all; % limpa toda a memória
% Posicao inicial do robô bidimensional: P = (x,y,th)
% Pode ser qualquer uma. Vamos escolher a origem só por conveniência.
P = [0, % x inicial do robô [m]
    0, % y inicial do robô [m]
    0]; % th inicial do robô [rad]
% Parâmetros do robô, obtidos do manual do fabricante, ou então, medidos
% diretamente no robô
% Raio da roda (r) [m]
r = 1/2 * (195/1000); % [m]
% Metade do comprimento do eixo das rodas (l) [m]
l = 1/2 * (381/1000); % [m]
% Lei de controle: velocidade das rodas do robô
% Para colocar o robô em movimento, deve-se, de fato, mudar as velocidades
% das duas rodas de acordo com a conveniência que se queira.
% Assim, determinar qual velocidade cada roda deve ter é o que, realmente,
% faz o robô cumprir uma determinada tarefa que se queira.
% No caso deste exemplo, escolheremos o valor que quisermos para ver os
% efeitos. Isso é interessante no início para ganhar a sensibilidade sobre
% como o robô responde a diferentes estímulos. Faça testes com diferentes
% situações para ver como o robô se comporta.
% Velocidade angular da roda esquerda (dphi_l = wl) [rad/s] (Escolha:)
wl = deg2rad(30); % [rad/s]
% Velocidade angular da roda direita (dphi_r = wr) [rad/s] (Escolha:)
wr = deg2rad(60); % [rad/s]
% Cálculo da velocidade do ponto-médio (robô).
% Utilizando as relações desenvolvidas na aula, calcula-se o par (v,w) do
% ponto-médio, o ponto que generaliza todas as partes do robô.
% Velocidade linear (velocidade de translação) [m/s]:
v = r/2 * (wr + wl); % [m/s]
% Velocidade angular (velocidade de rotação) [rad/s]:
w = r/(2*l) * (wr - wl); % [rad/s]
% Tempo de amostragem da simulação. O intervalo de tempo entre um instante
% e outro a ser considerado na simulação.
dt = 0.25; % [s]
% Armazenamento das posições do robô. Histórico de todas as posições que o
% robô passou, desde o início (instante t=0) até o momento atual (t).
R = P; % armazenamento do histórico de posições do robô.
% Tempo máximo de simulação [s]. Mude para o tempo que quiser.
tmax = 30; % [s]
% Simulação de velocidade linear por uma quantidade de tempo
for t = 0:dt:tmax % para cada um dos instantes de tempo 't'
    % Posição angular (theta) atual (conhecida) do robô. Ou seja, a posição
    % atual é a última posição conhecida, a posição do instante anterior.
    % th(t-1):
    th = P(3); % Esta linha não é necessária, somente por motivos didáticos.
    % No caso deste exemplo, as velocidades (v,w) serão constantes,
    % portanto não há necessidade de calculá-las aqui. Vamos deixá-las
    % aqui somente por motivos didáticos. No entanto, em programas com
    % algoritmos de controle, ou com joysticks (teleoperação), elas mudam
    % em todos os instantes de tempo 't', e é necessário obtê-las sempre.
    % Velocidade linear (velocidade de translação) [m/s]:
    v = r/2 * (wr + wl); % [m/s]
    % Velocidade angular (velocidade de rotação) [rad/s]:
    w = r/(2*l) * (wr - wl); % [rad/s]
    % Cálculo da variação da velocidade no instante de tempo 't' atual
    % Utilizando (v,w) obtidos, decompõe-se ambas em todos os eixos do
    % frame inercial, de modo a ter a contribuição de cada um deles.
    dP = [v * cos(th),           % v_x(t) é a decomposição em x de v(t)
          v * sin(th),           % v_y(t) é a decomposição em y de v(t)
          w];                    % w(t) é o próprio w(t)

    % Calcula a nova posição do robô no tempo 't' atual
    % Para atualizar a posição atual P(t-1) para a posição nova P(t), é
    % necessário integrar (somar) as contribuições de cada uma das
    % velocidades na posição. A velocidade é a variação da posição no
    % tempo, assim, a cada instante 'dt', tem-se sua contribuição.
    % Assim: P_nova = P_atual + dP_novo * dt
    P = P + dP * dt;
    % Armazena as posições do robô ao longo do tempo (histórico)
    R = [R , P];                % o histórico será útil para o plot dos dados.
    % É possível armazenar tudo diretamente em 'P', porém, por motivos
    % didáticos, resolvi separar o armazenamento, para clarificar.
    % ----- Plot dos resultados -----
    % Histórico de posições: o 'rastro' do robô, por onde ele passou.
    plot(R(1,:) , R(2,:) , 'b' , 'linewidth' , 2); hold on;
    % Posição atual do robô: onde ele está no momento atual 't'
    plot(P(1) , P(2) , 'or' , 'linewidth' , 2 , 'markersize' , 15)
    % Orientação atual do robô: para onde ele aponta no momento atual 't'
    plot([P(1) P(1)+0.1*cos(P(3))] , [P(2) P(2)+0.1*sin(P(3))] , 'r' , 'linewidth' , 2)
    hold off;axis equal; xlabel('x [m]'); ylabel('y [m]');grid on;
    drawnow
end