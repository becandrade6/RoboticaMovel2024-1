import math
import numpy as np
from pidController import pidController

class Robot2:
    def __init__(self,x=0,y=0,theta=0,linear_velocity=0,angular_velocity=0,wheel_radius=0,wheel_distance_to_center=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.wheel_radius = wheel_radius
        self.wheel_distance_to_center = wheel_distance_to_center

    def set_position(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

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
    
    def simulate_pid_controller(self, dt, goal, start, pid_gains):
        # Define PID gains
        kp, ki, kd = pid_gains

        # Define goal
        target_x, target_y, target_theta = goal

        self.set_position(start[0], start[1], start[2])

        # Initialize PID controller
        pid = pidController(kp, ki, kd)

        # Initialize error variables
        prev_error_x = 0.0
        prev_error_y = 0.0

