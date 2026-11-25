import gtsam
import numpy as np
from read_file3d import leer_g2o, matriz_info
from batch_solution3d import graficar, crear_pose


def incremental(poses, edges):
    params = gtsam.ISAM2Params()
    params.setFactorization("QR")
    isam = gtsam.ISAM2(params)
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3]))

    for fila in poses:
        pid = int(fila[0])
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()
        aristas = edges[edges[:, 1] == pid]

        if pid == 0:
            pose = crear_pose(fila[1:])
            graph.add(gtsam.PriorFactorPose3(pid, pose, prior_noise))
        else:
            edge = aristas[0]
            anterior = resultado.atPose3(int(edge[0]))
            pose = anterior.compose(crear_pose(edge[2:9]))

        initial.insert(pid, pose)

        for edge in aristas:
            i, j = int(edge[0]), int(edge[1])
            informacion = matriz_info(edge[9:30])
            # GTSAM usa rotación antes que traslación.
            orden = [3, 4, 5, 0, 1, 2]
            informacion = informacion[np.ix_(orden, orden)]
            noise = gtsam.noiseModel.Gaussian.Information(informacion)
            graph.add(gtsam.BetweenFactorPose3(i, j, crear_pose(edge[2:9]), noise))

        isam.update(graph, initial)
        resultado = isam.calculateEstimate()

    return resultado


if __name__ == "__main__":
    poses, edges = leer_g2o("parking-garage.g2o")
    resultado = incremental(poses, edges)
    print("Poses optimizadas:", resultado.size())
    graficar(poses, resultado, "resultados/ejercicio_3_3.png")
