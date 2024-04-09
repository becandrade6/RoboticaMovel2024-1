import numpy as np
import numpy as np
import sympy as sp
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Define the Van der Pol ODE function
def vanderpol_ode(t, y, mu):
    dydt = [y[1], mu * (1 - y[0] ** 2) * y[1] - y[0]]
    return dydt

# Define the initial conditions
y0 = [5, 2]
mu = 0.2

# Define the time span
t_span = (0, 30)

# Solve the Van der Pol ODE
sol = solve_ivp(lambda t, y: vanderpol_ode(t, y, mu), t_span, y0)

# Print the solution
print("The solution of the Van der Pol ODE is:")
print(sol.y[0])

# Create a figure with two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

# Plot the solution in the first subplot in blue
ax1.plot(sol.t, sol.y[0], color='blue', linewidth=2)
ax1.set_xlabel('Time')
ax1.set_ylabel('Solution')
ax1.set_title('Solution of the Van der Pol ODE')
ax1.grid(True)

# Plot the solution in the second subplot in red
ax2.plot(sol.t, sol.y[1], color='red', linewidth=2)
ax2.set_xlabel('Time')
ax2.set_ylabel('Solution')
ax2.grid(True)

# Adjust the spacing between subplots
plt.tight_layout()

# Save the figure
plt.savefig('/home/levty/Documentos/UFJF/9 Periodo/Robotica Movel/Aula 1 - Revisao Python/vanderPolSolution.png')

# Plot the phase diagram
fig, ax = plt.subplots(figsize=(6, 6))
ax.plot(sol.y[0], sol.y[1], color='green', linewidth=2)
ax.set_xlabel('Solution 1')
ax.set_ylabel('Solution 2')
ax.set_title('Phase Diagram of the Van der Pol ODE')
ax.grid(True)
# Save the phase diagram
plt.savefig('/home/levty/Documentos/UFJF/9 Periodo/Robotica Movel/Aula 1 - Revisao Python/vanderPolPhaseDiagram.png')