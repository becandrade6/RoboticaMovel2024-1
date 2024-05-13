import serial
import time
import struct

portaSerial = '/dev/ttyUSB0'

esp =  serial.Serial(
    port= portaSerial,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

velocidadeDireita = 0.2
velocidadeEsquerda = 0.2

tempo = 0
contador = 0
time.sleep(2)

startTime = time.time()
while tempo < 10:
    dt = time.time() - startTime
    if dt > 0.2:
        tempo += dt
        startTime = time.time()
        mensagem = bytearray([0x01]) + bytearray(struct.pack('<f',velocidadeDireita)) + bytearray(struct.pack('<f',velocidadeEsquerda)) 
        esp.write(mensagem)
        print("mensagem enviada")
        contador += 1
    time.sleep(0.01)

esp.close()