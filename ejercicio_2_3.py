import gtsam
import numpy as np
from read_file2d import leer_g2o, matriz_info
from batch_solution import graficar


def incremental(poses, edges):
    params = gtsam.ISAM2Params()
    params.setFactorization("QR")
    isam = gtsam.ISAM2(params)
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3, 1e-4]))

    for fila in poses:
        pid = int(fila[0])
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()
        aristas = edges[edges[:, 1] == pid]

        if pid == 0:
            pose = gtsam.Pose2(*fila[1:])
            graph.add(gtsam.PriorFactorPose2(pid, pose, prior_noise))
        else:
            edge = aristas[0]
            anterior = resultado.atPose2(int(edge[0]))
            pose = anterior.compose(gtsam.Pose2(*edge[2:5]))

        initial.insert(pid, pose)

        for edge in aristas:
            i, j = int(edge[0]), int(edge[1])
            informacion = matriz_info(edge[5:11])
            noise = gtsam.noiseModel.Gaussian.Information(informacion)
            graph.add(gtsam.BetweenFactorPose2(i, j, gtsam.Pose2(*edge[2:5]), noise))

        isam.update(graph, initial)
        resultado = isam.calculateEstimate()

    return resultado


def main():
    poses, edges = leer_g2o("input_INTEL_g2o.g2o")
    resultado = incremental(poses, edges)
    print("Poses optimizadas:", resultado.size())
    graficar(poses, resultado, "resultados/ejercicio_2_3.png")
