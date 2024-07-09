from refatorado import *
Ktry = np.array([1.0, 1.0, 1.0],float)
dp = np.array([0.1,0.1,0.1],float)

K, bestError = twiddle()
#Kteste = np.array([0.449,0.0,4.035],float)
simulate_and_plot(K,'CaminhoPercorrido.png')