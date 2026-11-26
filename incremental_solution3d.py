import gtsam
import numpy as np
from read_file3d import leer_g2o, matriz_info
from graficos import graficar_3d

DATA = "data/parking-garage.g2o"
SALIDA = "resultados/incremental3d.png"

def crear_pose(valores):
    x, y, z, qx, qy, qz, qw = valores
    q = np.array([qw, qx, qy, qz])
    q = q / np.linalg.norm(q)
    rotacion = gtsam.Rot3.Quaternion(*q)
    return gtsam.Pose3(rotacion, np.array([x, y, z]))


def incremental(poses, edges):
    params = gtsam.ISAM2Params()
    params.setFactorization("QR")
    isam = gtsam.ISAM2(params)
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3])
    )

    for fila in poses:
        pid = int(fila[0])
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()

        if pid == 0:
            pose = crear_pose(fila[1:])
            graph.add(gtsam.PriorFactorPose3(pid, pose, prior_noise))
            initial.insert(pid, pose)
        else:
            arista_anterior = next(edge for edge in edges if int(edge[1]) == pid and int(edge[0]) < pid)
            pose_anterior = resultado.atPose3(int(arista_anterior[0]))
            medicion = crear_pose(arista_anterior[2:9])
            pose = pose_anterior.compose(medicion)
            initial.insert(pid, pose)

        for edge in edges:
            i, j = int(edge[0]), int(edge[1])
            if j == pid:
                informacion = matriz_info(edge[9:30])
                orden = [3, 4, 5, 0, 1, 2]
                informacion = informacion[np.ix_(orden, orden)]
                noise = gtsam.noiseModel.Gaussian.Information(informacion)
                graph.add(gtsam.BetweenFactorPose3(i, j, crear_pose(edge[2:9]), noise))

        isam.update(graph, initial)
        resultado = isam.calculateEstimate()
        error = isam.getFactorsUnsafe().error(resultado)
        print(f"Pose {pid}: error acumulado = {error}")

    return resultado


def main():
    poses, edges = leer_g2o(DATA)
    resultado = incremental(poses, edges)
    print("Poses optimizadas:", resultado.size())
    graficar_3d(poses, resultado, SALIDA)


if __name__ == "__main__":
    main()
