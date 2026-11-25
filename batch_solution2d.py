import gtsam
import matplotlib.pyplot as plt
import numpy as np
from read_file2d import leer_g2o, matriz_info


def crear_grafo(poses, aristas):
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    for pid, x, y, theta in poses:
        initial.insert(int(pid), gtsam.Pose2(x, y, theta))

    pid0 = int(poses[0, 0])
    
    # prior_noise representa cuánto error permitimos en la pose inicial que usamos como referencia.
    # elegimos números pequeños
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3, 1e-4]))
    
    graph.add(gtsam.PriorFactorPose2(pid0, initial.atPose2(pid0), prior_noise))

    for arista in aristas:
        i, j = int(arista[0]), int(arista[1])
        informacion = matriz_info(arista[5:11])
        noise = gtsam.noiseModel.Gaussian.Information(informacion)
        graph.add(gtsam.BetweenFactorPose2(i, j, gtsam.Pose2(*arista[2:5]), noise))

    return graph, initial


def perturbar(initial, primer_id, desviacion=0.1):
    rng = np.random.default_rng(0)
    perturbado = gtsam.Values()

    for pid in initial.keys():
        pose = initial.atPose2(pid)
        if pid == primer_id:
            perturbado.insert(pid, pose)
        else:
            ruido = rng.normal(0, desviacion, 3)
            perturbado.insert(pid, gtsam.Pose2(
                pose.x() + ruido[0], pose.y() + ruido[1], pose.theta() + ruido[2]
            ))

    return perturbado


def gauss_newton(graph, initial, iteraciones=10, min_error=1e-3):
    params = gtsam.GaussNewtonParams()
    params.setLinearSolverType("MULTIFRONTAL_QR")
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)

    print("Error inicial:", graph.error(initial))
    if  graph.error(initial) <= min_error:
        return initial

    for i in range(iteraciones):
        optimizer.iterate()
        error = optimizer.error()
        print("Iteración", i + 1, "error:", error)
        if error <= min_error:
            break

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
    poses, aristas = leer_g2o("input_INTEL_g2o.g2o")

    graph, initial = crear_grafo(poses, aristas)

    initial = perturbar(initial, int(poses[0, 0]))
    resultado = gauss_newton(graph, initial, min_error=110)

    graficar(poses, resultado)
