from robot import Robot
import math
import numpy as np
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


# body =  [[100 , 227.5 , 227.5 , 100 , -200 , -227.5 , -227.5 , -200],[-190.5 , -50 , 50 , 190.5 , 190.5 , 163 , -163 , -190.5],[1, 1, 1, 1, 1, 1, 1, 1]]
# leftWheel = [[97.5, 97.5, -97.5, -97.5],[170.5, 210.5, 210.5, 170.5],[1, 1, 1, 1]]
# rightWheel =  [[97.5, 97.5, -97.5, -97.5],[-170.5, -210.5, -210.5, -170.5],[1, 1, 1, 1]]

# for i in range(2):
#     for j in range(7):
#         body[i][j] = body[i][j]/1000
#     for j in range(4):
#         leftWheel[i][j] = leftWheel[i][j]/1000
#         rightWheel[i][j] = rightWheel[i][j]/1000


x0 = 0
y0 = 0 
theta0 = 0
print("Raio de curvatura: ", R)
print("tempo maximo de simulação: ", tmax)
robo = Robot(wheelRadius = r, wheelDistanceToCenter = l, x = x0, y = y0, theta = theta0, linearVelocity = v, angularVelocity = w)

robo.differentialSimulation(dt1, tmax, 'differentialSimulation.png')
robo.set_position(x0, y0, theta0)
robo.differentialSimulation(dt2, tmax, 'differentialSimulation2.png')
robo.set_position(x0, y0, theta0)
robo.incrementalSimulation(dt1, tmax, 'incrementalSimulation.png')
robo.set_position(x0, y0, theta0)
robo.incrementalSimulation(dt2, tmax, 'incrementalSimulation2.png')
#robo.differentialSimulationWithGeometry(dt1, tmax, 'differentialSimulationWithGeometry.png')