"""
Ejercicios 2.1 y 3.1: lectura de archivos G2O.

Formato 2D:
    VERTEX_SE2 id x y theta
    EDGE_SE2 i j dx dy dtheta q11 q12 q13 q22 q23 q33

Formato 3D:
    VERTEX_SE3:QUAT id x y z qx qy qz qw
    EDGE_SE3:QUAT i j x y z qx qy qz qw + 21 números de información
"""
import numpy as np


def leer_g2o_2d(path):
    """
    Devuelve:
      poses: array N x 4  -> (id, x, y, theta)
      edges: array M x 11 -> (i, j, dx, dy, dtheta, q11, q12, q13, q22, q23, q33)
    """
    return _leer_g2o(path, "VERTEX_SE2", 4, "EDGE_SE2", 11)


def leer_g2o_3d(path):
    """
    Devuelve:
      poses: array N x 8  -> (id, x, y, z, qx, qy, qz, qw)
      edges: array M x 30 -> (i, j, x, y, z, qx, qy, qz, qw, 21 de información)
    """
    return _leer_g2o(path, "VERTEX_SE3:QUAT", 8, "EDGE_SE3:QUAT", 30)


def _leer_g2o(path, tag_vertice, n_vertice, tag_arista, n_arista):
    poses, edges = [], []
    with open(path) as archivo:
        for linea in archivo:
            campos = linea.split()
            if not campos:
                continue
            if campos[0] == tag_vertice:
                poses.append([float(v) for v in campos[1:1 + n_vertice]])
            elif campos[0] == tag_arista:
                edges.append([float(v) for v in campos[1:1 + n_arista]])
    poses = np.array(poses)
    edges = np.array(edges)
    poses = poses[np.argsort(poses[:, 0])]  # ordenadas por id
    return poses, edges


def matriz_info_2d(q):
    """Matriz de información 3x3 a partir de su triángulo superior (6 números)."""
    q11, q12, q13, q22, q23, q33 = q
    return np.array([[q11, q12, q13],
                     [q12, q22, q23],
                     [q13, q23, q33]])


def matriz_info_3d(q):
    """Matriz de información 6x6 a partir de su triángulo superior (21 números, fila por fila)."""
    omega = np.zeros((6, 6))
    k = 0
    for i in range(6):
        for j in range(i, 6):
            omega[i, j] = q[k]
            omega[j, i] = q[k]
            k += 1
    return omega


def info_a_covarianza(omega):
    """Covarianza = inversa de la información (para el informe; GTSAM usa Ω directo)."""
    return np.linalg.inv(omega)
