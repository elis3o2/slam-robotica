import gtsam
import numpy as np
import matplotlib.pyplot as plt
from read_file2d import leer_g2o, matriz_info


def incremental(poses, edges):
    params = gtsam.ISAM2Params()
    params.setFactorization("QR")
    isam = gtsam.ISAM2(params)
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3, 1e-4]))

    for pid, x, y, theta in poses:
        pid = int(pid)
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()

        if pid == 0:
            pose = gtsam.Pose2(x, y, theta)
            graph.add(gtsam.PriorFactorPose2(pid, pose, prior_noise))
            initial.insert(pid, pose)
        else:
            arista_anterior = next(edge for edge in edges if int(edge[1]) == pid and int(edge[0]) < pid)
            pose_anterior = resultado.atPose2(int(arista_anterior[0]))
            medicion = gtsam.Pose2(*arista_anterior[2:5])
            initial.insert(pid, pose_anterior.compose(medicion))

        for edge in edges:
            i, j = int(edge[0]), int(edge[1])
            if j == pid:
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
    puntos = np.array([resultado.atPose2(int(p[0])).translation() for p in poses])
    plt.plot(poses[:, 1], poses[:, 2], label="Inicial")
    plt.plot(puntos[:, 0], puntos[:, 1], label="Optimizada")
    plt.axis("equal")
    plt.legend()
    plt.savefig("ejercicio_2_3.png")
    plt.show()


if __name__ == "__main__":
    main()
