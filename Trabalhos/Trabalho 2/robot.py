import matplotlib.pyplot as plt
import math
import numpy as np
#raio da roda
#distancia do centro a roda
#posicao x, y, theta iniciais
#velocidade angular da roda esquerda e direita iniciais

#geometry = {
#   'body': [[],[],[]]
#   'leftWheel': [[],[],[]]
#   'rightWheel': [[],[],[]]
#}

class Robot:
    def __init__(self, wheelRadius = 0 , wheelDistanceToCenter = 1,
                 x=0, y=0, theta=0, 
                 leftWheelAngularSpeed=None, rightWheelAngularSpeed=None, linearVelocity=None, angularVelocity=None,
                 body = None, leftWheel = None, rightWheel = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.wheelDistanceToCenter = wheelDistanceToCenter
        self.wheelRadius = wheelRadius
        if(leftWheelAngularSpeed == None and rightWheelAngularSpeed == None):
            self.linearVelocity = linearVelocity
            self.angularVelocity = angularVelocity
            self.calculateWheelSpeedsFromVelocity()
        elif (linearVelocity == None and angularVelocity == None):
            self.leftWheelAngularSpeed = leftWheelAngularSpeed
            self.rightWheelAngularSpeed = rightWheelAngularSpeed
            self.calculateVelocityFromWheelSpeeds()
        self.body = body
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel

    def set_position(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    # def set_geometry(self, body, leftWheel, rightWheel):
    #     self.body = body
    #     self.leftWheel = leftWheel
    #     self.rightWheel = rightWheel

    def get_position(self):
        return [self.x, self.y, self.theta]
    
    # def get_geometry(self):
    #     return [self.body, self.leftWheel, self.rightWheel]

    def set_wheel_speeds(self, leftWheelAngularSpeed, rightWheelAngularSpeed):
        self.leftWheelAngularSpeed = leftWheelAngularSpeed
        self.rightWheelAngularSpeed = rightWheelAngularSpeed

    def set_speeds(self, linearVelocity, angularVelocity):
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity

    def calculateWheelSpeedsFromVelocity(self):
        self.rightWheelAngularSpeed = (self.linearVelocity + self.angularVelocity * self.wheelDistanceToCenter)/(self.wheelRadius)
        self.leftWheelAngularSpeed = (self.linearVelocity - self.angularVelocity * self.wheelDistanceToCenter)/(self.wheelRadius)

    def calculateVelocityFromWheelSpeeds(self):
        v = (self.wheelRadius)/2 * (self.rightWheelAngularSpeed + self.leftWheelAngularSpeed)
        omega = ((self.wheelRadius)/(2*self.wheelDistanceToCenter))* (self.rightWheelAngularSpeed - self.leftWheelAngularSpeed)
        self.linearVelocity = v
        self.angularVelocity = omega
        return v, omega

    def differentialMovement(self, dt):
        v, omega = self.calculateVelocityFromWheelSpeeds()
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += omega * dt
    
    def incrementalMovement(self, dt):
        v, omega = self.calculateVelocityFromWheelSpeeds()
        deltaS = v * dt
        deltaTheta = omega * dt
        deltaP = [deltaS * math.cos(self.theta + deltaTheta/2), deltaS * math.sin(self.theta + deltaTheta/2), deltaTheta]
        self.set_position(self.x + deltaP[0], self.y + deltaP[1], self.theta + deltaP[2])

    # def change_geometry(self, body, leftWheel, rightWheel):
    #     newBody = self.translation2D(self.rotation2D(body,self.theta), self.x, self.y) 
    #     newLeftWheel = self.translation2D(self.rotation2D(leftWheel,self.theta), self.x, self.y)
    #     newRightWheel = self.translation2D(self.rotation2D(rightWheel,self.theta), self.x, self.y)
    #     self.set_geometry(newBody, newLeftWheel, newRightWheel)
        

    def differentialSimulation(self, dt, maxTime,pathToFigure):
        time = 0
        history = [self.get_position()]
        while time <= maxTime:
            self.differentialMovement(dt)
            time += dt
            history.append(self.get_position())
        self.plot(values = history,pathToFigure=pathToFigure)
    
    # def differentialSimulationWithGeometry(self,dt,maxTime,pathToFigure):
    #     time = 0
    #     body, leftWheel, rightWheel = self.get_geometry()
    #     historyGeometry = [[body, leftWheel, rightWheel]]
    #     historyPosition = [self.get_position()]
    #     while time <= maxTime:
    #         self.differentialMovement(dt)
    #         self.change_geometry(body, leftWheel, rightWheel)
    #         time += dt
    #         historyGeometry.append(self.get_geometry())
    #         historyPosition.append(self.get_position())
    #     self.plotWithGeometry(geometryValues = historyGeometry,positionValues = historyPosition,pathToFigure=pathToFigure)

    def incrementalSimulation(self,dt,maxTime,pathToFigure):
        time = 0
        history = [self.get_position()]
        while time <= maxTime:
            self.incrementalMovement(dt)
            time += dt
            history.append(self.get_position())
        self.plot(values = history,pathToFigure=pathToFigure)
    

    def plot(self,values,pathToFigure):
        #adaptar para perfumaria a partir do codigo do olivi
        # values = [ [x1, y1, theta1], [x2, y2, theta2], ...]
        x_values = [value[0] for value in values]
        y_values = [value[1] for value in values]
        theta_values = [value[2] for value in values]
        plt.plot(x_values, y_values)
        plt.quiver(x_values, y_values, [math.cos(theta) for theta in theta_values], [math.sin(theta) for theta in theta_values])
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Robot Trajectory')
        plt.grid()
        plt.savefig(pathToFigure)
        plt.close()

    # def plotWithGeometry(self,geometryValues, positionValues,pathToFigure):
        
    #     bodyValues = [value[0] for value in geometryValues]
    #     leftWheelValues = [value[1] for value in geometryValues]
    #     rightWheelValues = [value[2] for value in geometryValues]
        
    #     bodyValues[0] = np.array(bodyValues[0])
    #     leftWheelValues[0] = np.array(leftWheelValues[0])
    #     rightWheelValues[0] = np.array(rightWheelValues[0])        
        
    #     xBody_values = [value[0] for value in bodyValues]
    #     yBody_values = [value[1] for value in bodyValues]
    #     xLeftWheel_values = [value[0,:] for value in leftWheelValues]
    #     yLeftWheel_values = [value[1,:] for value in leftWheelValues]
    #     xRightWheel_values = [value[0,:] for value in rightWheelValues]
    #     yRightWheel_values = [value[1,:] for value in rightWheelValues]


    #     plt.fill(xBody_values,yBody_values, 'y')  # corpo
    #     plt.fill(xLeftWheel_values, yLeftWheel_values, 'y')  # roda esquerda
    #     plt.fill(xRightWheel_values, yRightWheel_values, 'y')  # roda direita
        
    #     xPosition_values = [value[0] for value in positionValues]
    #     yPosition_values = [value[1] for value in positionValues]
    #     thetaPosition_values = [value[2] for value in positionValues]

    #     plt.plot(xPosition_values, yPosition_values, 'b', linewidth=1)


    #     primeiroEixoX = []
    #     primeiroEixoY = []
    #     segundoEixoX = []
    #     segundoEixoY = []
    #     for i in range(len(xPosition_values)):
    #         primeiroEixoX.append([xPosition_values[i], xPosition_values[i] + self.wheelDistanceToCenter * np.cos(thetaPosition_values[i] + np.pi / 2)])
    #         primeiroEixoY.append([yPosition_values[i], yPosition_values[i] + self.wheelDistanceToCenter * np.sin(thetaPosition_values[i] + np.pi / 2)])
    #         segundoEixoX.append([xPosition_values[i], xPosition_values[i] + self.wheelDistanceToCenter * np.cos(thetaPosition_values[i] - np.pi / 2)])
    #         segundoEixoY.append([yPosition_values[i], yPosition_values[i] + self.wheelDistanceToCenter * np.sin(thetaPosition_values[i] - np.pi / 2)])

    #     #eixo das rodas
    #     plt.plot(primeiroEixoX,primeiroEixoY, 'k', linewidth=1.5)
    #     plt.plot(segundoEixoX,segundoEixoY, 'k', linewidth=1.5)

    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.title('Robot Trajectory')
    #     plt.grid()
    #     plt.savefig(pathToFigure)
    #     plt.close()
    
    # def translation2D(self, originalFrameMatrix, deltaX, deltaY):
    #     T = [[1, 0, deltaX], [0, 1, deltaY], [0, 0, 1]]
    #     translatedMatrix = T @ originalFrameMatrix
    #     return translatedMatrix
    
    # def rotation2D(self, originalFrameMatrix, theta):
    #     Rz = [[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]]
    #     rotatedMatrix = np.dot(np.array(Rz),np.array(originalFrameMatrix))
    #     return rotatedMatrix
    