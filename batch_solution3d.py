import gtsam
import numpy as np
import argparse
from graficos import graficar_3d
from read_file3d import leer_g2o, matriz_info

DATA = "data/parking-garage.g2o"
SALIDA = "resultados/batch3d.png"
SALIDA_PERTURBADA = "resultados/batch3d_perturbado.png"


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
    
    # Fijamos la primer posicion
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3]))
    graph.add(gtsam.PriorFactorPose3(pid0, initial.atPose3(pid0), prior_noise))

    for arista in aristas:
        i, j = int(arista[0]), int(arista[1])
        informacion = matriz_info(arista[9:30])
        # GTSAM usa rotación antes que traslación.
        # reordenamos las filas y columnas según orden 
        orden = [3, 4, 5, 0, 1, 2]
        informacion = informacion[np.ix_(orden, orden)]
        
        noise = gtsam.noiseModel.Gaussian.Information(informacion)
        graph.add(gtsam.BetweenFactorPose3(i, j, crear_pose(arista[2:9]), noise))

    return graph, initial


def perturbar(initial, primer_id, desviacion=0.1):
    rng = np.random.default_rng(0)
    perturbado = gtsam.Values()

    for pid in initial.keys():
        pose = initial.atPose3(pid)
        if pid == primer_id:
            perturbado.insert(pid, pose)
        else:
            ruido = rng.normal(0, desviacion, 6)
            perturbado.insert(pid, pose.retract(ruido))

    return perturbado


def gauss_newton(graph, initial, iteraciones=100, min_error=1e-3):
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



def main():
    parser = argparse.ArgumentParser(description="SLAM 3D por lotes")
    parser.add_argument("--perturbar", action="store_true", help="Perturbar las poses iniciales")
    args = parser.parse_args()

    poses, aristas = leer_g2o(DATA)
    graph, initial = crear_grafo(poses, aristas)
    
    if args.perturbar:
        initial = perturbar(initial, int(poses[0, 0]))
    resultado = gauss_newton(graph, initial)
    
    salida = SALIDA_PERTURBADA if args.perturbar else SALIDA
    graficar_3d(poses, resultado, salida, inicial=initial)


if __name__ == "__main__":
    main()
