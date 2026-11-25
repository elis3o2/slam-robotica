"""
Graph SLAM 3D (ejercicios 3.1, 3.2 y 3.3) con el dataset parking-garage.

Uso:
    python slam_3d.py [archivo.g2o]
"""
import sys
import matplotlib.pyplot as plt

from lectura_g2o import leer_g2o_3d, matriz_info_3d
from batch import crear_grafo, gauss_newton, levenberg_marquardt
from incremental import isam2
from modelo import posiciones
from graficos import graficar_3d

DIM = 3


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "parking-garage.g2o"

    # 3.1 Lectura
    poses, edges = leer_g2o_3d(path)
    print(f"Leídos {len(poses)} vértices y {len(edges)} aristas de {path}")
    print("Matriz de información de la primera arista (orden g2o):")
    print(matriz_info_3d(edges[0, 9:30]))

    ids = poses[:, 0]
    inicial = poses[:, 1:4]

    # 3.2 Batch
    graph, initial = crear_grafo(poses, edges, DIM)
    gn = gauss_newton(graph, initial)
    graficar_3d(inicial, posiciones(gn, ids, DIM),
                "Graph SLAM 3D - Batch (Gauss-Newton)", "resultados/batch_gn_3d.png")

    lm = levenberg_marquardt(graph, initial)
    graficar_3d(inicial, posiciones(lm, ids, DIM),
                "Graph SLAM 3D - Batch (Levenberg-Marquardt)", "resultados/batch_lm_3d.png")

    # 3.3 Incremental
    inc = isam2(poses, edges, DIM)
    graficar_3d(inicial, posiciones(inc, ids, DIM),
                "Graph SLAM 3D - Incremental (iSAM2)", "resultados/incremental_3d.png")

    plt.show()


if __name__ == "__main__":
    main()
