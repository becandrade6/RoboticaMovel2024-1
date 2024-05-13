import matplotlib.pyplot as plt
import math
import numpy as np

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
    
    def get_position(self):
        return [self.x, self.y, self.theta]

    def getCenterSpeeds(self):
        return self.linearVelocity, self.angularVelocity
    
    def getWheelSpeeds(self):
        return self.leftWheelAngularSpeed, self.rightWheelAngularSpeed

    def set_wheel_speeds(self, leftWheelAngularSpeed, rightWheelAngularSpeed):
        self.leftWheelAngularSpeed = leftWheelAngularSpeed
        self.rightWheelAngularSpeed = rightWheelAngularSpeed
        self.calculateVelocityFromWheelSpeeds()

    def set_speeds(self, linearVelocity, angularVelocity):
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
        self.calculateWheelSpeedsFromVelocity()

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
        v, omega = self.getCenterSpeeds()
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += omega * dt

    def incrementalMovement(self, dt):
        v, omega = self.calculateVelocityFromWheelSpeeds()
        deltaS = v * dt
        deltaTheta = omega * dt
        deltaP = [deltaS * math.cos(self.theta + deltaTheta/2), deltaS * math.sin(self.theta + deltaTheta/2), deltaTheta]
        self.set_position(self.x + deltaP[0], self.y + deltaP[1], self.theta + deltaP[2])
        
    def differentialSimulation(self, dt, maxTime,pathToFigure):
        time = 0
        history = [self.get_position()]
        while time <= maxTime:
            self.differentialMovement(dt)
            time += dt
            history.append(self.get_position())
        self.plot(values = history,pathToFigure=pathToFigure,hasRadius=True)

    def incrementalSimulation(self,dt,maxTime,pathToFigure):
        time = 0
        history = [self.get_position()]
        while time <= maxTime:
            self.incrementalMovement(dt)
            time += dt
            history.append(self.get_position())
        self.plot(values = history,pathToFigure=pathToFigure,hasRadius=True)
    
    
    def proportionalControllerSimulation(self,dt,pathToFigure,goals,parameters):
        delta1 = parameters['delta1']
        delta2 = parameters['delta2']
        history = [self.get_position()]
        for goal in goals:
            deltaX, deltaY, deltaTheta = self.calculatePositionDifference(goal)
            rho, gamma, alpha, beta = self.calculateParametersToGoal(goal)
            while (abs(rho) > delta1) and (abs(deltaTheta) > delta2):
                v, w = self.calculateControlSpeeds(goal,parameters)
                self.set_speeds(v, w)
                self.differentialMovement(dt)
                history.append(self.get_position())
                rho, gamma, alpha, beta = self.calculateParametersToGoal(goal)
        goals.insert(0,(0,0,np.deg2rad(90)))
        self.plot(values = history,pathToFigure=pathToFigure,goals=goals)

    def adjustAngle(self,angle):
        angle = angle % (2 * math.pi)
        if angle > math.pi:
            angle  = angle - 2 * math.pi
        return angle
    
    def calculatePositionDifference(self,goal):
        position = self.get_position()
        dX = goal[0] - position[0]
        dY = goal[1] - position[1]
        dTheta = self.adjustAngle(goal[2] - position[2])
        return dX, dY, dTheta

    def calculateParametersToGoal(self,goal):
        dX, dY, dTheta = self.calculatePositionDifference(goal)
        rho = math.sqrt(dX**2 + dY**2)
        gamma = self.adjustAngle(math.atan2(dY,dX))
        alpha = self.adjustAngle(gamma - self.theta)
        beta = self.adjustAngle(goal[2] - gamma)
        if((-180 <= math.degrees(alpha) < -90) or (90 < math.degrees(alpha) <= 180)):
            v,w = self.getCenterSpeeds()
            self.set_speeds(-v,w)
            alpha = self.adjustAngle(alpha + math.pi)
            beta = self.adjustAngle(beta + math.pi)
        return rho, gamma, alpha, beta

    def calculateProportionalControlSpeeds(self,goal,parameters):
        Krho = parameters['Krho']
        Kalpha = parameters['Kalpha']
        Kbeta = parameters['Kbeta']
        vmax = parameters['vmax']
        wmax = parameters['wmax']
        rho, gamma, alpha, beta = self.calculateParametersToGoal(goal)
        v = Krho * rho if abs(Krho * rho) < abs(vmax) else vmax
        w = Kalpha * alpha + Kbeta * beta if abs(Kalpha * alpha + Kbeta * beta) < abs(wmax) else wmax
        return v, w

    def twiddlePIDAlgorithm(self,dt,goal,pidParameters,dp,start,adjustRate = 0.1,tolerance = 0.01,tMax = 5,vMax = 0.5,wMax = 180):
        bestError,firstTrajectory = self.pidProcessSimulation(dt,goal,pidParameters,tolerance,tMax,vMax,wMax,start)
        errorHistory = [bestError]
        trajectoryHistory = [firstTrajectory]
        n = len(pidParameters)
        while sum(abs(x) for x in dp) > tolerance:
            for i in range(n):
                pidParameters[i] += dp[i]
                error, trajectory = self.pidProcessSimulation(dt,goal,pidParameters,tolerance,tMax,vMax,wMax,start)
                errorHistory.append(error)
                trajectoryHistory.append(trajectory)
                if error < bestError:
                    bestError = error
                    dp[i] *= (1 + adjustRate)
                else:
                    pidParameters[i] -= 2*dp[i]
                    error,trajectory = self.pidProcessSimulation(dt,goal,pidParameters,tolerance,tMax,vMax,wMax,start)
                    errorHistory.append(error)
                    trajectoryHistory.append(trajectory)
                    if error < bestError:
                        bestError = error
                        dp[i] *= (1 + adjustRate)
                    else:
                        pidParameters[i] += dp[i]
                        dp[i] *= (1 - adjustRate)
        return bestError, errorHistory,pidParameters
        

    def pidProcessSimulation(self,dt,goal,pidParameters,tolerance, tMax,vMax,wMax,start):
        Kp, Ki, Kd = pidParameters
        erros = []
        posHistory = [start]
        totalTime = 0
        alphas = [0]

        x0, y0, theta0 = start
        x,y,theta = x0, y0, theta0
        xg, yg, thetag = goal
        distance = math.sqrt((xg - x)**2 + (yg - y)**2)

        while distance > tolerance or totalTime < tMax:
            totalTime += dt
            x, y, theta = self.get_position()
            distance = math.sqrt((xg - x)**2 + (yg - y)**2)
            alpha = self.adjustAngle(math.atan2(yg - y, xg - x) - theta)
            v = min(distance, vMax)
            w = min(Kp*alpha + Ki * sum(alphas) + Kd * (alpha - alphas[-1]), wMax)
            alphas.append(alpha)
            erros.append(self.calculateError([x0,y0,theta0],[x,y,theta],goal))
            self.set_speeds(v,w)
            self.differentialMovement(dt)
            posHistory.append(self.get_position())
        
        return sum(erros), posHistory
    
    #TODO: reformular o calculateError para calcular o erro conforme formula dada certinha
    #devemos acho que usar uma nova função de simulação e nao apenas o calculateError cpa
    def calculateError(self,start,actualPosition,goal):
        x0, y0, theta0 = start
        xg, yg, thetag = goal
        x, y, theta = actualPosition

        erro = abs((y0 - yg)*x + (xg - x0)*y + (x0*yg - xg*y0))/math.sqrt((x0 - xg)**2 + (yg - y0)**2)
        return erro


    def plot(self,values,pathToFigure,hasRadius = False,goals = None):
        # values = [ [x1, y1, theta1], [x2, y2, theta2], ...]
        x_values = [value[0] for value in values]
        y_values = [value[1] for value in values]
        theta_values = [value[2] for value in values]
        
        plt.figure(figsize=(10, 8))  
        plt.plot(x_values, y_values)
        if(goals != None):
            for goal in goals:
                plt.quiver(goal[0], goal[1], [math.cos(goal[2])], [math.sin(goal[2])], color='red')
                plt.scatter(goal[0], goal[1], color='k')
        #plt.quiver(x_values, y_values, [math.cos(theta) for theta in theta_values], [math.sin(theta) for theta in theta_values])
        plt.xlabel('Eixo X')
        plt.ylabel('Eixo Y')
        if(hasRadius):
            radius = max(max(x_values) - min(x_values), max(y_values) - min(y_values)) / 2
            plt.title('Trajetória do Robô com raio simulado de : ' + str(radius), weight='bold')
        plt.grid()
        plt.savefig(pathToFigure)
        plt.close()