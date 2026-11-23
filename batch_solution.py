import gtsam
import matplotlib.pyplot as plt
import numpy as np
from read_file2d import leer_g2o, matriz_info


def crear_grafo(poses, edges):
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    for pid, x, y, theta in poses:
        initial.insert(int(pid), gtsam.Pose2(x, y, theta))

    pid0 = int(poses[0, 0])
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3, 1e-4]))
    graph.add(gtsam.PriorFactorPose2(pid0, initial.atPose2(pid0), prior_noise))

    for edge in edges:
        i, j = int(edge[0]), int(edge[1])
        informacion = matriz_info(edge[5:11])
        noise = gtsam.noiseModel.Gaussian.Information(informacion)
        graph.add(gtsam.BetweenFactorPose2(i, j, gtsam.Pose2(*edge[2:5]), noise))

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


def graficar(poses, resultado):
    puntos = np.array([resultado.atPose2(int(p[0])).translation() for p in poses])
    plt.figure(figsize=(8, 7))
    plt.plot(poses[:, 1], poses[:, 2], label="Inicial")
    plt.plot(puntos[:, 0], puntos[:, 1], label="Optimizada")
    plt.axis("equal")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig("ejercicio_2_2.png")
    plt.show()
    plt.close()



def main():
    poses, edges = leer_g2o("input_INTEL_g2o.g2o")

    graph, initial = crear_grafo(poses, edges)

    resultado = optimizar_gauss_newton(graph, initial)

    graficar(poses, resultado)
