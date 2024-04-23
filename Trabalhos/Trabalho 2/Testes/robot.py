import matplotlib.pyplot as plt
import math

#raio da roda
#distancia do centro a roda
#posicao x, y, theta iniciais
#velocidade angular da roda esquerda e direita iniciais

#nomeVariavel = {
#   'body': []
#   'leftWheel': []
#   'rightWheel': []
#}

class Robot:
    def __init__(self, wheelRadius = 0 , wheelDistanceToCenter = 1,
                 x=0, y=0, theta=0, 
                 leftWheelAngularSpeed=None, rightWheelAngularSpeed=None, linearVelocity=None, angularVelocity=None,
                 geometry = None):
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
        self.geometry = geometry

    def set_position(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

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
        v, omega = self.calculate_velocity()
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += omega * dt

    def differentialSimulation(self, dt, maxTime,pathToFigure):
        time = 0
        history = [[self.x, self.y, self.theta]]
        while time <= maxTime:
            self.differentialMovement(dt)
            history.append([self.x, self.y, self.theta])
            time += dt
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
        plt.savefig(pathToFigure)
        plt.close()