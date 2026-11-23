import numpy as np


def matriz_info(q):
    matriz = np.zeros((6, 6))
    k = 0
    for i in range(6):
        for j in range(i, 6):
            matriz[i, j] = q[k]
            matriz[j, i] = q[k]
            k += 1
    return matriz


def leer_g2o(path):
    poses = []
    edges = []

    with open(path, "r") as archivo:
        for linea in archivo:
            campos = linea.strip().split()
            if not campos:
                continue

            tipo = campos[0]
            if tipo == "VERTEX_SE3:QUAT":
                valores = list(map(float, campos[1:9]))
                poses.append(valores)
            elif tipo == "EDGE_SE3:QUAT":
                valores = list(map(float, campos[1:31]))
                edges.append(valores)

    return np.array(poses), np.array(edges)
