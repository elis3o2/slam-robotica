"""
Graph SLAM 2D (ejercicios 2.1, 2.2 y 2.3) con el dataset INTEL.

Uso:
    python slam_2d.py [archivo.g2o]
"""
import sys
import matplotlib.pyplot as plt

from lectura_g2o import leer_g2o_2d, matriz_info_2d
from batch import crear_grafo, gauss_newton, levenberg_marquardt
from incremental import isam2
from modelo import posiciones
from graficos import graficar_2d

DIM = 2


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "input_INTEL_g2o.g2o"

    # 2.1 Lectura
    poses, edges = leer_g2o_2d(path)
    print(f"Leídos {len(poses)} vértices y {len(edges)} aristas de {path}")
    print("Matriz de información de la primera arista:")
    print(matriz_info_2d(edges[0, 5:11]))

    ids = poses[:, 0]
    inicial = poses[:, 1:3]

    # 2.2 Batch
    graph, initial = crear_grafo(poses, edges, DIM)
    gn = gauss_newton(graph, initial)
    graficar_2d(inicial, posiciones(gn, ids, DIM),
                "Graph SLAM 2D - Batch (Gauss-Newton)", "resultados/batch_gn_2d.png")

    lm = levenberg_marquardt(graph, initial)
    graficar_2d(inicial, posiciones(lm, ids, DIM),
                "Graph SLAM 2D - Batch (Levenberg-Marquardt)", "resultados/batch_lm_2d.png")

    # 2.3 Incremental
    inc = isam2(poses, edges, DIM)
    graficar_2d(inicial, posiciones(inc, ids, DIM),
                "Graph SLAM 2D - Incremental (iSAM2)", "resultados/incremental_2d.png")

    plt.show()


if __name__ == "__main__":
    main()
