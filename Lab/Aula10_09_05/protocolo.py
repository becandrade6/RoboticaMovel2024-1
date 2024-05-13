import serial
import time
import struct
import csv

portaSerial = '/dev/ttyUSB0'

esp =  serial.Serial(
    port= portaSerial,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

velocidades = [0.2,0.1,0.3,0.4,0.1]
#velocidadeDireita = 0.2
#velocidadeEsquerda = 0.2
#fazer vetor de velocidades para ele ir alterando de acordo com a desejada
dataTotalEsquerdo = []
dataTotalDireito = []
tempo = 0
contador = 0
time.sleep(2)

startTime = time.time()
for velocidade in velocidades:
    velocidadeDireita = velocidade
    velocidadeEsquerda = velocidade        
    while tempo < 4:
        dt = time.time() - startTime
        if dt > 0.2:
            tempo += dt
            startTime = time.time()
            mensagem = bytearray([0x01]) + bytearray(struct.pack('<f',velocidadeDireita)) + bytearray(struct.pack('<f',velocidadeEsquerda)) 
            esp.write(mensagem)
            print("mensagem enviada")
            if esp.in_waiting > 0:
                dataEsquerda = esp.read()
                dataDireita = esp.read()
                print(dataEsquerda)
                print(dataDireita)
                dataTotalEsquerdo.append(dataEsquerda)
                dataTotalDireito.append(dataDireita)
                print("mensagem recebida")
            contador += 1
        time.sleep(0.01)

esp.close()

# Salve os valores de dataTotalEsquerdo em um arquivo CSV
with open('/home/levty/Documentos/UFJF/9_Periodo/Robotica Movel/Github-Repo/Lab/Aula10_09_05/dataTotalEsquerdo.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(dataTotalEsquerdo)

# Salve os valores de dataTotalDireito em um arquivo CSV
with open('/home/levty/Documentos/UFJF/9_Periodo/Robotica Movel/Github-Repo/Lab/Aula10_09_05/dataTotalDireito.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(dataTotalDireito)