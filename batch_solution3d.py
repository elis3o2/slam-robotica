import gtsam
import matplotlib.pyplot as plt
import numpy as np
from read_file3d import leer_g2o, matriz_info


def crear_pose(valores):
    x, y, z, qx, qy, qz, qw = valores
    q = np.array([qw, qx, qy, qz])
    q = q / np.linalg.norm(q)
    rotacion = gtsam.Rot3.Quaternion(*q)
    return gtsam.Pose3(rotacion, np.array([x, y, z]))


def crear_grafo(poses, aristas):
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    for fila in poses:
        initial.insert(int(fila[0]), crear_pose(fila[1:]))

    pid0 = int(poses[0, 0])
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3]))
    graph.add(gtsam.PriorFactorPose3(pid0, initial.atPose3(pid0), prior_noise))

    for arista in aristas:
        i, j = int(arista[0]), int(arista[1])
        informacion = matriz_info(arista[9:30])
        # GTSAM usa rotación antes que traslación.
        orden = [3, 4, 5, 0, 1, 2]
        informacion = informacion[np.ix_(orden, orden)]
        noise = gtsam.noiseModel.Gaussian.Information(informacion)
        graph.add(gtsam.BetweenFactorPose3(i, j, crear_pose(arista[2:9]), noise))

    return graph, initial


def optimizar_gauss_newton(graph, initial, iteraciones=10):
    params = gtsam.GaussNewtonParams()
    params.setLinearSolverType("MULTIFRONTAL_QR")
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)

    print("Error inicial:", graph.error(initial))
    for i in range(iteraciones):
        optimizer.iterate()
        print("Iteración", i + 1, "error:", optimizer.error())

    return optimizer.values()


def graficar(poses, resultado, salida="resultados/ejercicio_3_2.png"):
    puntos = np.array([resultado.atPose3(int(p[0])).translation() for p in poses])
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(poses[:, 1], poses[:, 2], poses[:, 3], label="Inicial")
    ax.plot(puntos[:, 0], puntos[:, 1], puntos[:, 2], label="Optimizada")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(salida)
    plt.show()
    plt.close()


if __name__ == "__main__":
    poses, aristas = leer_g2o("parking-garage.g2o")
    graph, initial = crear_grafo(poses, aristas)
    resultado = optimizar_gauss_newton(graph, initial)
    graficar(poses, resultado)
