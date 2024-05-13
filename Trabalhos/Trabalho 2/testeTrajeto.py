import matplotlib.pyplot as plt
from robot import Robot
import math
import numpy as np

v = 0
w = 0

dt1 = 69/350
dt2 = 69/7000

r = 0.5* (195/1000)
l = 0.5* (381/1000)


x0 = 1.5
y0 = 0.0
theta0 = math.atan2(1.5 - 0,1.5 - 1.5)

letterB = [(1.5,0, math.atan2(1.5 - 0,1.5 - 1.7)),(1.5,1.5, math.atan2(3 - 1.5,1.5 - 1.7)), (1.5,3, math.atan2(4.5 - 3,1.5 - 1.8)),(1.5,4.5,math.atan2(4.5 - 4.5,2.5-1.5)), (2.5,4.5, math.atan2(4 - 4.5,3.5-2.5)), (3.5,4, math.atan2(3.25 - 4,3.25-3.5)), (3.25,3.25, math.atan2(3 - 3.25,3-3.25)), (3,3, math.atan2(2.73 - 3,2.5 - 3)), (2.5,2.73, math.atan2(2.71 - 2.73,2 - 2.5)), (2,2.71, math.atan2(2.5 - 2.71,2.5 - 2.0)), (2.5,2.5, math.atan2(2.23 - 2.5, 3 - 2.5)), (3,2.23, math.atan2(1.86 - 2.23, 3.25 - 3)),(3.25,1.86,math.atan2(1.27 - 1.86, 3.5-3.25)), (3.5,1.27, math.atan2(0.75 - 1.27, 3.24 -3.5)),(3.24,0.75,math.atan2(0.5 - 0.75,3-3.24)), (3,0.5, math.atan2(0 - 0.5, 2-3)), (2,0,math.atan2(0 - 0, 1.5-2)), (1.5,0, math.atan2(0 - 0, 0-1.5))]
letterL = [(1,4, math.atan2(2 - 4,1 - 1)),(1,2, math.atan2(0 - 2,1 - 1.2)),(1,0, math.atan2(0-0,3 - 1)),(3,0, np.deg2rad(0))]
letterV = [(0,4, math.atan2(2 - 4,1 - 0)),(1,2, math.atan2(0 - 2,2 - 1)),(2,0, math.atan2(2 - 0,3 - 2)), (3,2, math.atan2(4 - 2,5 - 4)), (4,4, np.deg2rad(90))]

parameters = {'delta1': 0.01, 'delta2': np.deg2rad(1), 'Krho': (2/11)  , 'Kalpha': (450/823), 'Kbeta': (-2/11), 'vmax': 0.4, 'wmax' : np.deg2rad(300)}

# robo = Robot(wheelRadius = r, wheelDistanceToCenter = l, x = x0, y = y0, theta = theta0, linearVelocity = v, angularVelocity = w)
# robo.proportionalControllerSimulation(dt1,'controlB_desequilibrio.png',letterB,parameters)

# robo = Robot(wheelRadius = r, wheelDistanceToCenter = l, x = 1.0, y = 4.0, theta = math.atan2(2 - 4, 0), linearVelocity = v, angularVelocity = w)
# robo.proportionalControllerSimulation(dt1,'controlL.png',letterL,parameters)

robo = Robot(wheelRadius = r, wheelDistanceToCenter = l, x = 0.0, y = 4.0, theta = math.atan2(0 - 4,2.5 - 0), linearVelocity = v, angularVelocity = w)
robo.proportionalControllerSimulation(dt1,'controlV3.png',letterV,parameters)



