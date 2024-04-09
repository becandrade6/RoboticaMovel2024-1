import os
import pandas as pd
import PyPDF2
import tabula

# Diretório dos arquivos PDF
diretorio = '/caminho/para/os/arquivos/pdf'

# Lista para armazenar os números de conta
numeros_conta = []

# Percorre todos os arquivos PDF no diretório
for arquivo in os.listdir(diretorio):
    if arquivo.endswith('.pdf'):
        # Abre o arquivo PDF
        with open(os.path.join(diretorio, arquivo), 'rb') as file:
            reader = PyPDF2.PdfReader(file)
            
            # Extrai as tabelas do arquivo PDF
            tabelas = tabula.read_pdf(file, pages='all')
            
            # Percorre todas as tabelas extraídas
            for tabela in tabelas:
                # Verifica se a tabela contém a coluna "Numero da conta"
                if 'Numero da conta' in tabela.columns:
                    # Adiciona os números de conta à lista
                    numeros_conta.extend(tabela['Numero da conta'].tolist())

# Cria um DataFrame com os números de conta
df = pd.DataFrame({'Numero da conta': numeros_conta})

# Exporta o DataFrame para uma planilha do Excel
df.to_excel('/caminho/para/a/planilha.xlsx', index=False)