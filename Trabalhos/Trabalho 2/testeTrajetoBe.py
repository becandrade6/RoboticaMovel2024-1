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


x0 = 0.0
y0 = 0.0
theta0 = np.deg2rad(90)

#goal = [(1.5,0, np.deg2rad(0)),(1.5, 4.5,np.deg2rad(90)), (2.5,4.5, np.deg2rad(0)), (3.5,4, np.deg2rad(45)), (3.25,3.25, np.deg2rad(-45))]
#goal = [(1.5,0, np.deg2rad(0)),(1.5, 4.5,np.deg2rad(90)), (2.5,4.5, np.deg2rad(0))]
goal = [(0,4.5, math.atan2(4.5 - 4.5,1.5 - 0)),(1.5,4.5,math.atan2(4.5 - 4.5,1.5-0))]

parameters = {'delta1': 0.1, 'delta2': np.deg2rad(5), 'Krho': 2/11, 'Kalpha': 345/823, 'Kbeta': -2/11, 'vmax': 0.4, 'wmax' : np.deg2rad(300)}

robo = Robot(wheelRadius = r, wheelDistanceToCenter = l, x = x0, y = y0, theta = theta0, linearVelocity = v, angularVelocity = w)
robo.proportionalControllerSimulation(dt1,'control.png',goal,parameters)