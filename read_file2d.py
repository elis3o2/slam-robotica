import numpy as np


def matriz_info(q):
    q11, q12, q13, q22, q23, q33 = q
    return np.array([
        [q11, q12, q13],
        [q12, q22, q23],
        [q13, q23, q33]
    ])


def leer_g2o(path):
    poses = []
    aristas = []

    with open(path, "r") as archivo:
        for linea in archivo:
            campos = linea.strip().split()
            if not campos:
                continue

            tipo = campos[0]
            if tipo == "VERTEX_SE2":
                valores = list(map(float, campos[1:5]))
                poses.append(valores)
            elif tipo == "EDGE_SE2":
                valores = list(map(float, campos[1:12]))
                aristas.append(valores)

    return np.array(poses), np.array(aristas)
