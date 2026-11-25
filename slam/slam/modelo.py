"""
Traducción de los datos del .g2o a objetos de GTSAM, para 2D y 3D.

batch.py e incremental.py solo usan estas funciones, así que no necesitan
saber si el problema es 2D o 3D: eso se decide con el parámetro `dim`.
"""
import numpy as np
import gtsam

from lectura_g2o import matriz_info_2d, matriz_info_3d


# Ruido del prior sobre la primera pose: muy chico, la deja prácticamente fija.
#   2D: (x, y, theta)
#   3D: (rotación x3, traslación x3), que es el orden que usa GTSAM en Pose3
SIGMAS_PRIOR = {
    2: np.array([1e-3, 1e-3, 1e-4]),
    3: np.array([1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3]),
}

# g2o ordena el error 3D como (traslación, rotación) y GTSAM como
# (rotación, traslación): hay que intercambiar los bloques de 3x3 de Ω.
ORDEN_G2O_A_GTSAM = [3, 4, 5, 0, 1, 2]


def crear_pose(valores, dim):
    """
    2D: valores = (x, y, theta)
    3D: valores = (x, y, z, qx, qy, qz, qw)
    """
    if dim == 2:
        return gtsam.Pose2(*valores)

    x, y, z, qx, qy, qz, qw = valores
    q = np.array([qw, qx, qy, qz])      # GTSAM pide el cuaternión como (w, x, y, z)
    q = q / np.linalg.norm(q)           # normalizado, por si viene con redondeo
    return gtsam.Pose3(gtsam.Rot3.Quaternion(*q), np.array([x, y, z]))


def pose_de_fila(fila, dim):
    """Pose a partir de una fila del array `poses` (la columna 0 es el id)."""
    return crear_pose(fila[1:], dim)


def factor_prior(pid, pose, dim):
    ruido = gtsam.noiseModel.Diagonal.Sigmas(SIGMAS_PRIOR[dim])
    if dim == 2:
        return gtsam.PriorFactorPose2(pid, pose, ruido)
    return gtsam.PriorFactorPose3(pid, pose, ruido)


def factor_arista(edge, dim):
    """BetweenFactor a partir de una fila del array `edges`."""
    i, j = int(edge[0]), int(edge[1])

    if dim == 2:
        medicion = crear_pose(edge[2:5], dim)
        omega = matriz_info_2d(edge[5:11])
    else:
        medicion = crear_pose(edge[2:9], dim)
        omega = matriz_info_3d(edge[9:30])
        omega = omega[np.ix_(ORDEN_G2O_A_GTSAM, ORDEN_G2O_A_GTSAM)]

    # Las Ω de estos datasets están muy mal condicionadas: se pasan directo
    # como información, sin invertirlas a covarianza (evita perder precisión).
    ruido = gtsam.noiseModel.Gaussian.Information(omega)

    if dim == 2:
        return gtsam.BetweenFactorPose2(i, j, medicion, ruido)
    return gtsam.BetweenFactorPose3(i, j, medicion, ruido)


def obtener_pose(values, k, dim):
    return values.atPose2(int(k)) if dim == 2 else values.atPose3(int(k))


def posiciones(values, ids, dim):
    """Array N x 2 (2D) o N x 3 (3D) con las posiciones de las poses."""
    if dim == 2:
        return np.array([[p.x(), p.y()] for p in (values.atPose2(int(k)) for k in ids)])
    return np.array([[p.x(), p.y(), p.z()] for p in (values.atPose3(int(k)) for k in ids)])
