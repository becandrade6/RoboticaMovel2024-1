import serial
import random
import numpy as np

# Função para enviar comandos para o ESP
def enviar_comando(comando):
    esp.write(comando.encode())

# Defina o nome da porta serial utilizada
porta_serial = '/dev/ttyUSB0'  # Substitua pelo nome da porta serial correta

esp =  serial.Serial(
    port=porta_serial,
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

try:
    numeroAmostrasAleatorias = 50
    numeroRepetirPWM = 5
    dt = 0.1
    n = [random.randint(0, 255) for _ in range(numeroAmostrasAleatorias)]
    u = []
    for valor in n:
        u.extend([valor] * numeroRepetirPWM)
    # Enviar cada elemento do array u para o ESP via serial e aguardar a resposta
    d = []
    for valor in u:
        enviar_comando(str(valor))
        valor_entrada = esp.readline().decode().strip()
        encoder_roda1 = esp.readline().decode().strip()
        encoder_roda2 = esp.readline().decode().strip()
        
        d.append([int(valor_entrada), float(encoder_roda1), float(encoder_roda2),dt])

    header = "valor_entrada, encoder_roda1, encoder_roda2, dt"
    np.savetxt('/home/levty/Documentos/UFJF/9_Periodo/Lab Robotica Movel/Aula05_18_04/t100a01.csv', d, delimiter=',', header=header)
    esp.close()
except Exception as e:
    esp.close()
    print(f"Ocorreu um erro: {e}")