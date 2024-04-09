# Aula 04: Modelos Cinemáticos - Abordagem diferencial

## Introdução

Robô tem duas rodas independentes.
Todos os objetos em cena possuem seus respectivos frames.

O robô é composto de várias partes e essas partes também podem ter seus próprios frames.

Por isso, toda cena terá um frame inercial (frame parado de referência).

Em robótica terrestre, o convencional é que o eixo x seja a frente e o eixo z aponte para cima. Nas aulas adotaremos o padrão convencional:

![alt text](image.png)

O robô terá o seu próprio referencial, porém a posição espacial do robô (P) no mundo bidimensional será um vetor que conta com as suas coordenadas:

$$
P =
\begin{bmatrix}
x\\
y\\
\end{bmatrix}
$$

É necessário dizer a orientação do robô no mundo, acrescentando em P o angulo $\theta$ de orientação, logo $P = [x, y, \theta]^{T}$

![alt text2](image-1.png)

## Quais variáveis temos?

Temos as velocidades de rotação das duas rodas. E sabemos as dimensões físicas do robô.

Sendo $r $ o raio da roda do robô, w a velocidade angular temos que a velocidade linear é:

$ v = r\*w$

Cada uma das duas rodas contribui com _metade_ da velocidade linear do ponto médio, já que este ncontra-se na exata metade do caminho.Logo, a soma das duas contribuições é:

$v= \frac{r}{2}*(\omega{r}+\omega{l})$ , sendo r de right e l de left

## Velocidades de modo geral

De modo geral, sendo r o raio da roda e l a distância da roda até o ponto médio do robô temos que:

![alt text 3](image-2.png)

e inversamente:

![alt text 4](image-3.png)

## Modelo cinemático no frame global

Tendo as relações de velocidade,obtemos:

![alt text 5](image-4.png)

Reescrevendo, temos:

![alt text 6](image-5.png)

Podemos então obter a posição pela integração numérica da velocidade.

![alt text 7](image-6.png)

# Abordagem incremental

Seja um robô se movendo sob um circulo, temos o seguinte:

![alt text](image-7.png)

Assim, basicamente calculamos o deslocamento a partir do theta do instante anterior mais a variação entre os theta dividido por 2.

Então, temos:

![alt text](image-8.png)

Em formato matricial:

![alt text](image-9.png)

Para o caso de obtermos o $\Delta{S}$ das duas rodas de forma distinta,onde r é right e l é left:

![alt text](image-10.png)

ou em velocidades lineares:

![alt text](image-11.png)
