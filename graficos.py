import matplotlib.pyplot as plt
import numpy as np


def graficar_2d(poses, resultado, salida, inicial=None):
    puntos = np.array([resultado.atPose2(int(p[0])).translation() for p in poses])
    origen = poses[:, 1:3] if inicial is None else np.array([
        inicial.atPose2(int(p[0])).translation() for p in poses
    ])
    plt.figure(figsize=(8, 7))
    plt.plot(origen[:, 0], origen[:, 1], label="Sin Optimizar")
    plt.plot(puntos[:, 0], puntos[:, 1], label="Optimizada")
    plt.axis("equal")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(salida)
    plt.show()
    plt.close()



def graficar_3d(poses, resultado, salida, inicial=None):
    puntos = np.array([resultado.atPose3(int(p[0])).translation() for p in poses])
    origen = poses[:, 1:4] if inicial is None else np.array([
        inicial.atPose3(int(p[0])).translation() for p in poses
    ])
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(origen[:, 0], origen[:, 1], origen[:, 2], label="Sin Optimizar")
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
