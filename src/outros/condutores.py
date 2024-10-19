def calcular_reta_e_angulos(ponto_A, ponto_B):
    import numpy as np

    # Convertendo os pontos para arrays numpy
    A = np.array(ponto_A)
    B = np.array(ponto_B)

    # Vetor de direção da reta
    vetor_direcao = B - A

    # Cálculo dos ângulos de yaw, pitch e roll
    yaw = np.arctan2(vetor_direcao[1], vetor_direcao[0])
    pitch = np.arctan2(vetor_direcao[2], np.sqrt(vetor_direcao[0]**2 + vetor_direcao[1]**2))
    roll = 0  # Assumido como zero para esta orientação

    # Convertendo os ângulos de radianos para graus
    yaw_graus = np.degrees(yaw)
    pitch_graus = np.degrees(pitch)

    # Ajustando os ângulos conforme solicitado
    yaw_final = yaw_graus - 180
    pitch_final = pitch_graus - 90

    # Convertendo de volta para radianos
    yaw_final_rad = np.radians(yaw_final)
    pitch_final_rad = np.radians(pitch_final)

    # Cálculo do ponto central da reta
    ponto_central = (A + B) / 2

    # Cálculo do tamanho da reta
    tamanho_reta = np.linalg.norm(vetor_direcao)

    return {
        "Ponto Central": ponto_central,
        "Roll (rad)": roll,
        "Pitch (rad)": pitch_final_rad,
        "Yaw (rad)": yaw_final_rad,
        "Tamanho da Reta": tamanho_reta
    }

# Exemplo de uso com os pontos fornecidos
ponto_A = (-143.855820, 205.949341, 11.130083)
ponto_B = (-32.630505, 240.117950, 22.637791)


resultado = calcular_reta_e_angulos(ponto_A, ponto_B)
print(resultado)