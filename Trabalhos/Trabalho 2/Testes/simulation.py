from robot import Robot
import math
# 2020 69 002 B
# velocidade linear = 2020/2023 = 0.9985 m/s
#002 % pi/2 -> 0.42920367320510344
#dt1 = 69/350 = 0.19714285714285715
#dt2 = 69/7000 = 0.009857142857142857

v = 0.9985
w = 0.42920367320510344

dt1 = 69/350
dt2 = 69/7000
tmax = math.pi*2 / w
R = v/w
r = 0.5* (195/1000)
l = 0.5* (381/1000)

x0 = 0
y0 = 0 
theta0 = 0
print("Raio de curvatura: ", R)
print("tempo maximo de simulação: ", tmax)
robot = Robot(wheelRadius = r, wheelDistanceToCenter = l, x = x0, y = y0, theta = theta0, linearVelocity = v, angularVelocity = w)

robot.differentialSimulation(dt1, tmax, 'differentialSimulation.png')
robot.differentialSimulation(dt2, tmax, 'differentialSimulation2.png')