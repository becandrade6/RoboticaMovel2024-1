import numpy as np
import numpy as np
import sympy as sp
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Define the ODE function
def ode_func(t, y):
    return -2 * y

# Define the initial condition
y0 = 1

# Define the time span
t_span = (0, 5)

# Solve the ODE
sol = solve_ivp(ode_func, t_span, [y0])

# Print the solution
print("The solution of the ODE is:")
print(sol.y[0])