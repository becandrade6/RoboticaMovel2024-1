import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import sqrtm

# AjustaAngulo function equivalent in Python
def AjustaAngulo(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def ErrorF(K, pltS):
    global vetErro, vetTempo

    Kp, Ki, Kd = K
    Krho = 1

    Path = np.array([[1, 2, 2, 3, 4, 5, 5, 4, -4, -5, -5, -4, -3, -2, -2, -1, 0], 
                     [0, 1, 5, 6, 6, 5, 4, 3, 3, 4, 5, 6, 6, 5, 1, 0, 0]],float)

    P = np.array([0.0, 0.0, 0.0]) # x, y, th
    R = np.array(P, ndmin=2).T
    t = 0
    tmax = 80
    dt = 0.01
    delta = 0.05
    vmax = 1.0
    wmax = np.deg2rad(300)
    vetErro = []
    vetTempo = []

    erro_total = []

    for i in range(Path.shape[1]):
        dif_alpha = int_alpha = alpha_old = 0
        cont = 0
        G = Path[:, i]
        Pi = P if cont == 0 else Path[:, i-1]

        while True:
            t += dt
            Dx = G[0] - P[0]
            Dy = G[1] - P[1]
            rho = np.hypot(Dx, Dy)
            if rho <= delta or t > tmax:
                break

            gamma = AjustaAngulo(np.arctan2(Dy, Dx))
            alpha = AjustaAngulo(gamma - P[2])

            dif_alpha = alpha - alpha_old
            alpha_old = alpha
            int_alpha += alpha

            v = vmax if i < Path.shape[1] - 1 else min(Krho * rho, vmax)
            w = Kp * alpha + Ki * int_alpha + Kd * dif_alpha
            w = np.sign(w) * min(abs(w), wmax)

            dPdt = np.array([v * np.cos(P[2]), v * np.sin(P[2]), w])
            P += dPdt * dt
            P[2] = AjustaAngulo(P[2])

            R = np.column_stack((R, P))

            m = (G[1] - Pi[1]) / (G[0] - Pi[0]) if (G[0] - Pi[0]) != 0 else np.inf
            a, b, c = 1, -m, m * G[0] - G[1]

            if cont == 0:
                nErro = abs(a * P[0] + b * P[1] + c) / np.hypot(a, b)
                Erro = np.array([nErro])
                cont += 1
            else:
                nErro = abs(w) + abs(a * P[0] + b * P[1] + c) / np.hypot(a, b)
                Erro = np.append(Erro, nErro)

        erro_total.append(Erro)

    sumErro = np.mean(np.concatenate(erro_total))
    return sumErro

def twiddle(K = np.array([6.0, 6.0, 6.0],float),dp = np.array([0.6,0.6,0.6],float),tol=1e-3):
    # Inicialização do Twiddle
    iteracao = 0
    best_err = ErrorF(K, 0)

    while sum(dp) > tol:
        for i in range(len(K)):
            K[i] += dp[i]
            err = ErrorF(K, 0)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                K[i] -= 2 * dp[i]
                err = ErrorF(K, 0)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.05
                else:
                    K[i] += dp[i]
                    dp[i] *= 0.95
        iteracao += 1
        print("Iteração: ", iteracao)

    return K, best_err

def simulate_and_plot(K, path):
    Kp, Ki, Kd = K
    Krho = 1

    Path = np.array([[1, 2, 2, 3, 4, 5, 5, 4, -4, -5, -5, -4, -3, -2, -2, -1, 0], 
                     [0, 1, 5, 6, 6, 5, 4, 3, 3, 4, 5, 6, 6, 5, 1, 0, 0]])

    P = np.array([0.0, 0.0, 0.0]) # x, y, th
    R = np.array(P, ndmin=2).T
    t = 0
    tmax = 80
    dt = 0.01
    delta = 0.05
    vmax = 1.0
    wmax = np.deg2rad(300)

    for i in range(Path.shape[1]):
        dif_alpha = int_alpha = alpha_old = 0
        G = Path[:, i]

        while True:
            t += dt
            Dx = G[0] - P[0]
            Dy = G[1] - P[1]
            rho = np.hypot(Dx, Dy)
            if rho <= delta or t > tmax:
                break

            gamma = AjustaAngulo(np.arctan2(Dy, Dx))
            alpha = AjustaAngulo(gamma - P[2])

            dif_alpha = alpha - alpha_old
            alpha_old = alpha
            int_alpha += alpha

            v = vmax if i < Path.shape[1] - 1 else min(Krho * rho, vmax)
            w = Kp * alpha + Ki * int_alpha + Kd * dif_alpha
            w = np.sign(w) * min(abs(w), wmax)

            dPdt = np.array([v * np.cos(P[2]), v * np.sin(P[2]), w])
            P += dPdt * dt
            P[2] = AjustaAngulo(P[2])

            R = np.column_stack((R, P))

    plt.figure()
    plt.plot(Path[0, :], Path[1, :], 'g--', label='Caminho desejado')
    plt.plot(R[0, :], R[1, :], 'b-', label='Caminho percorrido')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Simulação com K = ' + str(K))
    plt.savefig(path)

def simulate_with_errors (K,path):
    Kp, Ki, Kd = K
    Krho = 1

    Kx = 0.07
    Ky = 0.18
    Ktheta = 0.13
    Kv = 0.05
    Kw = 0.05

    Path = np.array([[1, 2, 2, 3, 4, 5, 5, 4, -4, -5, -5, -4, -3, -2, -2, -1, 0], 
                     [0, 1, 5, 6, 6, 5, 4, 3, 3, 4, 5, 6, 6, 5, 1, 0, 0]])

    P = np.array([0.0, 0.0, 0.0]) # x, y, th
    R = np.array(P, ndmin=2).T

    P_r = P.copy()
    R_r = np.array(P, ndmin=2).T
    
    t = 0
    tmax = 80
    dt = 0.01
    delta = 0.05
    vmax = 1.0
    wmax = np.deg2rad(300)

    for i in range(Path.shape[1]):
        dif_alpha = int_alpha = alpha_old = 0
        G = Path[:, i]

        while True:
            t += dt
            Dx = G[0] - P[0]
            Dy = G[1] - P[1]
            rho = np.hypot(Dx, Dy)
            if rho <= delta or t > tmax:
                break

            gamma = AjustaAngulo(np.arctan2(Dy, Dx))
            alpha = AjustaAngulo(gamma - P[2])

            dif_alpha = alpha - alpha_old
            alpha_old = alpha
            int_alpha += alpha

            v = vmax if i < Path.shape[1] - 1 else min(Krho * rho, vmax)
            w = Kp * alpha + Ki * int_alpha + Kd * dif_alpha
            w = np.sign(w) * min(abs(w), wmax)

            Ds = v*dt
            Dth = w*dt

            sigma_x = Kx * Ds
            sigma_y = Ky * Ds
            sigma_theta = Ktheta * Dth
            sigma_v = Kv * Ds
            sigma_w = Kw * Dth
            
            Q = np.array([[sigma_x**2, 0, 0],[0, sigma_y**2, 0],[0, 0, sigma_theta**2]])
            M = np.array([[sigma_v**2, 0],[0, sigma_w**2]])

            r1 = np.dot(sqrtm(Q), np.random.randn(3,1)).flatten()
            r2 = np.dot(sqrtm(M), np.random.randn(2,1)).flatten()

            dPdt_real = np.array([(v + r2[0])*np.cos(P_r[2] + r1[2]), 
                                  (v + r2[0])*np.sin(P_r[2] + r1[2]), 
                                  w + r2[1]]).flatten()
            P_r += r1 + dPdt_real * dt

            R_r = np.column_stack((R_r, P_r))
            
            dPdt = np.array([v * np.cos(P[2]), v * np.sin(P[2]), w])
            P += dPdt * dt
            
            P[2] = AjustaAngulo(P[2])
            

            R = np.column_stack((R, P))

    plt.figure()
    plt.plot(Path[0, :], Path[1, :], 'g--', label='Caminho desejado')
    plt.plot(R[0, :], R[1, :], 'b-', label='Caminho percorrido sem erro')
    plt.plot(R_r[0, :], R_r[1, :], 'r-', label='Caminho percorrido com erro')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Simulação com K = ' + str(K))
    plt.savefig(path)

def simulate_with_sensor (K,path,sigma=0.25):
    Kp, Ki, Kd = K
    Krho = 1

    Kx = 0.07
    Ky = 0.18
    Ktheta = 0.13
    Kv = 0.05
    Kw = 0.05

    Path = np.array([[1, 2, 2, 3, 4, 5, 5, 4, -4, -5, -5, -4, -3, -2, -2, -1, 0], 
                     [0, 1, 5, 6, 6, 5, 4, 3, 3, 4, 5, 6, 6, 5, 1, 0, 0]])

    P = np.array([0.0, 0.0, 0.0]) # x, y, th
    R = np.array(P, ndmin=2).T

    P_r = P.copy()
    R_r = np.array(P, ndmin=2).T
    
    t = 0
    tmax = 80
    dt = 0.01
    delta = 0.05
    vmax = 1.0
    wmax = np.deg2rad(300)

    #erros do sensor
    sigma = 0.25
    #matriz de covariancias do sensor
    R_s = np.array([[sigma**2,0],[0,sigma**2]])
    #matriz do sensor, queremos atingir apenas x e y
    C = np.array([[1,0,0],[0,1,0]])
    #primeira leitura do sensor
    s = np.dot(C,P_r) + np.dot(sqrtm(R_s),np.random.randn(2,1)).flatten()
    #armazenamento para o plot
    S = np.array(s,ndmin=2).T

    for i in range(Path.shape[1]):
        dif_alpha = int_alpha = alpha_old = 0
        G = Path[:, i]

        while True:
            t += dt
            Dx = G[0] - P[0]
            Dy = G[1] - P[1]
            rho = np.hypot(Dx, Dy)
            if rho <= delta or t > tmax:
                break

            gamma = AjustaAngulo(np.arctan2(Dy, Dx))
            alpha = AjustaAngulo(gamma - P[2])

            dif_alpha = alpha - alpha_old
            alpha_old = alpha
            int_alpha += alpha

            v = vmax if i < Path.shape[1] - 1 else min(Krho * rho, vmax)
            w = Kp * alpha + Ki * int_alpha + Kd * dif_alpha
            w = np.sign(w) * min(abs(w), wmax)

            Ds = v*dt
            Dth = w*dt

            sigma_x = Kx * Ds
            sigma_y = Ky * Ds
            sigma_theta = Ktheta * Dth
            sigma_v = Kv * Ds
            sigma_w = Kw * Dth
            
            Q = np.array([[sigma_x**2, 0, 0],[0, sigma_y**2, 0],[0, 0, sigma_theta**2]])
            M = np.array([[sigma_v**2, 0],[0, sigma_w**2]])

            r1 = np.dot(sqrtm(Q), np.random.randn(3,1)).flatten()
            r2 = np.dot(sqrtm(M), np.random.randn(2,1)).flatten()

            dPdt_real = np.array([(v + r2[0])*np.cos(P_r[2] + r1[2]), 
                                  (v + r2[0])*np.sin(P_r[2] + r1[2]), 
                                  w + r2[1]]).flatten()
            P_r += r1 + dPdt_real * dt

            R_r = np.column_stack((R_r, P_r))

            s = np.dot(C,P_r) + np.dot(sqrtm(R_s),np.random.randn(2,1)).flatten()
            S = np.column_stack((S,s))
            
            dPdt = np.array([v * np.cos(P[2]), v * np.sin(P[2]), w])
            P += dPdt * dt
            
            P[2] = AjustaAngulo(P[2])
            

            R = np.column_stack((R, P))

    plt.figure()
    plt.plot(S[0, :], S[1, :], 'y-', label='Caminho percorrido com sensor')
    plt.plot(Path[0, :], Path[1, :], 'g--', label='Caminho desejado')
    plt.plot(R[0, :], R[1, :], 'b-', label='Caminho percorrido sem erro')
    plt.plot(R_r[0, :], R_r[1, :], 'r-', label='Caminho percorrido com erro')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Simulação com K = ' + str(K))
    plt.savefig(path)


            
            
            

           

    plt.figure()
    plt.plot(S[0, :], S[1, :], 'y-', label='Caminho percorrido com sensor')
    plt.plot(Path[0, :], Path[1, :], 'g--', label='Caminho desejado')
    plt.plot(R[0, :], R[1, :], 'b-', label='Caminho percorrido sem erro')
    plt.plot(R_r[0, :], R_r[1, :], 'r-', label='Caminho percorrido com erro')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Simulação com K = ' + str(K))
    plt.savefig(path)

def simulate_with_Kalman (K,path,sigma=0.25):
    Kp, Ki, Kd = K
    Krho = 1

    Kx = 0.07
    Ky = 0.18
    Ktheta = 0.13
    Kv = 0.05
    Kw = 0.05

    Path = np.array([[1, 2, 2, 3, 4, 5, 5, 4, -4, -5, -5, -4, -3, -2, -2, -1, 0], 
                     [0, 1, 5, 6, 6, 5, 4, 3, 3, 4, 5, 6, 6, 5, 1, 0, 0]])

    P = np.array([0.0, 0.0, 0.0]) # x, y, th
    R = np.array(P, ndmin=2).T

    P_r = P.copy()
    R_r = np.array(P, ndmin=2).T
    
    t = 0
    tmax = 80
    dt = 0.01
    delta = 0.05
    vmax = 1.0
    wmax = np.deg2rad(300)

    #erros do sensor
    sigma = 0.25
    #matriz de covariancias do sensor
    R_s = np.array([[sigma**2,0],[0,sigma**2]])
    #matriz do sensor, queremos atingir apenas x e y
    C = np.array([[1,0,0],[0,1,0]])
    #primeira leitura do sensor
    s = np.dot(C,P_r) + np.dot(sqrtm(R_s),np.random.randn(2,1)).flatten()
    #armazenamento para o plot
    S = np.array(s,ndmin=2).T

    for i in range(Path.shape[1]):
        dif_alpha = int_alpha = alpha_old = 0
        G = Path[:, i]

        while True:
            t += dt
            Dx = G[0] - P[0]
            Dy = G[1] - P[1]
            rho = np.hypot(Dx, Dy)
            if rho <= delta or t > tmax:
                break

            gamma = AjustaAngulo(np.arctan2(Dy, Dx))
            alpha = AjustaAngulo(gamma - P[2])

            dif_alpha = alpha - alpha_old
            alpha_old = alpha
            int_alpha += alpha

            v = vmax if i < Path.shape[1] - 1 else min(Krho * rho, vmax)
            w = Kp * alpha + Ki * int_alpha + Kd * dif_alpha
            w = np.sign(w) * min(abs(w), wmax)

            dPdt = np.array([v * np.cos(P[2]), v * np.sin(P[2]), w])
            P += dPdt * dt
            
            P[2] = AjustaAngulo(P[2])
            R = np.column_stack((R, P))

            F = np.array([[1, 0, -v*np.sin(P[2])*dt],
                          [0, 1, v*np.cos(P[2])*dt],
                          [0, 0, 1]])
            
            Gk = np.array([[np.cos(P[2])*dt, 0],
                           [np.sin(P[2])*dt, 0],
                           [0, dt]])

            Ds = v*dt
            Dth = w*dt

            sigma_x = Kx * Ds
            sigma_y = Ky * Ds
            sigma_theta = Ktheta * Dth
            sigma_v = Kv * Ds
            sigma_w = Kw * Dth
            
            Q = np.array([[sigma_x**2, 0, 0],[0, sigma_y**2, 0],[0, 0, sigma_theta**2]])
            M = np.array([[sigma_v**2, 0],[0, sigma_w**2]])
            
            if(i == 0):
                P_ = Q.copy()
            
            r1 = np.dot(sqrtm(Q), np.random.randn(3,1)).flatten()
            r2 = np.dot(sqrtm(M), np.random.randn(2,1)).flatten()

            dPdt_real = np.array([(v + r2[0])*np.cos(P_r[2] + r1[2]), 
                                  (v + r2[0])*np.sin(P_r[2] + r1[2]), 
                                  w + r2[1]]).flatten()
            P_r += r1 + dPdt_real * dt

            R_r = np.column_stack((R_r, P_r))

            s = np.dot(C,P_r) + np.dot(sqrtm(R_s),np.random.randn(2,1))
            S = np.column_stack((S,s))

            P_ = np.dot(np.dot(F,P_),F.T)+ np.dot(np.dot(Gk,M),Gk.T) + Q
            z = np.dot(C,P)
            K_k = np.dot(np.dot(P_,C.T),np.linalg.inv(np.dot(np.dot(C,P_),C.T) + R_s))

            P_ = np.dot((np.eye(3) - np.dot(K_k,C)),P_)

    plt.figure()
    plt.plot(S[0, :], S[1, :], 'y-', label='Caminho percorrido com sensor')
    plt.plot(Path[0, :], Path[1, :], 'g--', label='Caminho desejado')
    plt.plot(R[0, :], R[1, :], 'b-', label='Caminho percorrido sem erro')
    plt.plot(R_r[0, :], R_r[1, :], 'r-', label='Caminho percorrido com erro')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Simulação com K = ' + str(K))
    plt.savefig(path)


            
            
            

           

    plt.figure()
    plt.plot(S[0, :], S[1, :], 'y-', label='Caminho percorrido com sensor')
    plt.plot(Path[0, :], Path[1, :], 'g--', label='Caminho desejado')
    plt.plot(R[0, :], R[1, :], 'b-', label='Caminho percorrido sem erro')
    plt.plot(R_r[0, :], R_r[1, :], 'r-', label='Caminho percorrido com erro')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Simulação com K = ' + str(K))
    plt.savefig(path)