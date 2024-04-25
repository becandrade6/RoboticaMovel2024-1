import pandas as pd

# Ler o arquivo CSV
csvFiles = ['t100a01.csv', 't100a02.csv', 't100a03.csv', 't100a04.csv', 't100a05.csv', 't100a06.csv', 't100a07.csv', 't100a08.csv']

for csvFile in csvFiles:
    df = pd.read_csv(csvFile)

    # Converter os valores para inteiros
    df['# valor_entrada'] = df['# valor_entrada'].astype(int)
    df[' encoder_rodaEsquerda'] = df[' encoder_rodaEsquerda'].astype(int)
    df[' encoder_rodaDireita'] = df[' encoder_rodaDireita'].astype(int)
    df[' dt'] = 0.1

    # Salvar o arquivo CSV com os valores convertidos
    csvFileNameToSave = csvFile.replace('.csv', '_convertido.csv')
    df.to_csv(csvFileNameToSave, index=False)