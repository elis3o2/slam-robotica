import gtsam
import numpy as np
from read_file2d import leer_g2o, matriz_info
from graficos import graficar_2d

DATA = "data/input_INTEL_g2o.g2o"
SALIDA = "resultados/incremental2d.png"

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
            pose = pose_anterior.compose(medicion)
            initial.insert(pid, pose)

        for edge in edges:
            i, j = int(edge[0]), int(edge[1])
            if j == pid:
                informacion = matriz_info(edge[5:11])
                noise = gtsam.noiseModel.Gaussian.Information(informacion)
                graph.add(gtsam.BetweenFactorPose2(i, j, gtsam.Pose2(*edge[2:5]), noise))

        isam.update(graph, initial)
        resultado = isam.calculateEstimate()
        error = isam.getFactorsUnsafe().error(resultado)
        print(f"Pose {pid}: error acumulado = {error}")

    return resultado


def main():
    poses, edges = leer_g2o(DATA)
    resultado = incremental(poses, edges)
    graficar_2d(poses, resultado, SALIDA)


if __name__ == "__main__":
    main()


