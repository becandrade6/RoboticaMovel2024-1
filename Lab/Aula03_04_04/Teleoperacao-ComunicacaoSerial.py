import serial

# Defina o nome da porta serial utilizada
porta_serial = '/dev/ttyUSB0'  # Substitua pelo nome da porta serial correta

esp =  serial.Serial(
    port=porta_serial,
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)
# Função para enviar comandos para o ESP
def enviar_comando(comando):
    esp.write(comando.encode())

# Loop principal
while True:
    try:
        # Aguarde a entrada do usuário
        comando = input("Digite o comando (w, a, s, d, spacebar) ou q para sair: ")
        if comando == 'q':
            break
        # Envie o comando para o ESP
        enviar_comando(comando)
    except Exception as e:
        print(f"Ocorreu um erro: {e}")
        break

# Encerre a comunicação serialal
esp.close()