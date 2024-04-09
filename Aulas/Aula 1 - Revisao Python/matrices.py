import numpy as np
import numpy as np
import sympy as sp
import numpy as np
from scipy.integrate import solve_ivp

# Definindo a matriz de coeficientes
A = np.array([[2, 1, -1],
              [4, -1, 3],
              [1, 3, -2]])

# Definindo o vetor de termos independentes
b = np.array([1, 4, -3])

# Resolvendo o sistema linear
x = np.linalg.solve(A, b)

# Imprimindo a solução
print("A solução do sistema é:")
print(x)

# Calculando o determinante de A
det_A = np.linalg.det(A)
# Imprimindo o valor do determinante
print("O determinante de A é:", det_A)

# Definindo variáveis simbólicas
x, y, z = sp.symbols('x y z')

# Definindo uma matriz simbólica
B = sp.Matrix([[x, y, z],
               [2*x, y+z, 3*x],
               [x+y, 2*z, x-z]])

# Calculando a inversa da matriz simbólica
B_inv = B.inv()

# Imprimindo a matriz simbólica e sua inversa
print("Matriz B:")
print(B)
print("Inversa de B:")
print(B_inv)