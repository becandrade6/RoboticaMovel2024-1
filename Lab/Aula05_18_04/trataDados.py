import pandas as pd

# Ler o arquivo CSV
df = pd.read_csv('/home/levty/Documentos/UFJF/9_Periodo/Lab Robotica Movel/Aula05_18_04/t100a01_vFinal.csv')

# Converter os valores para inteiros
df['valor_entrada'] = df['valor_entrada'].astype(int)
df['encoder_rodaEsquerda'] = df['encoder_rodaEsquerda'].astype(int)
df['encoder_rodaDireita'] = df['encoder_rodaDireita'].astype(int)
df['dt'] = 0.1

# Salvar o arquivo CSV com os valores convertidos
df.to_csv('/home/levty/Documentos/UFJF/9_Periodo/Lab Robotica Movel/Aula05_18_04/t100a01_vFinalInteiro.csv', index=False)