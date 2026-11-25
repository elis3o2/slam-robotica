"""Gráficos de trayectoria inicial vs. optimizada, en 2D y 3D."""
import os
import matplotlib.pyplot as plt


def _guardar(fig, archivo):
    carpeta = os.path.dirname(archivo)
    if carpeta:
        os.makedirs(carpeta, exist_ok=True)
    fig.savefig(archivo, dpi=150)
    print(f"Guardado: {archivo}")


def graficar_2d(inicial, optimizada, titulo, archivo):
    """inicial y optimizada: arrays N x 2 con (x, y)."""
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(inicial[:, 0], inicial[:, 1], color="tab:red", lw=1, label="Inicial")
    ax.plot(optimizada[:, 0], optimizada[:, 1], color="tab:blue", lw=1, label="Optimizada")
    ax.axis("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(titulo)
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    _guardar(fig, archivo)


def graficar_3d(inicial, optimizada, titulo, archivo):
    """inicial y optimizada: arrays N x 3. Vista 3D + vista superior + vista lateral."""
    fig = plt.figure(figsize=(15, 5))

    ax = fig.add_subplot(1, 3, 1, projection="3d")
    ax.plot(*inicial.T, color="tab:red", lw=0.8, label="Inicial")
    ax.plot(*optimizada.T, color="tab:blue", lw=0.8, label="Optimizada")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title("Vista 3D")
    ax.legend()

    for idx, (a, b, nombre) in enumerate([(0, 1, "Vista superior (x-y)"),
                                          (0, 2, "Vista lateral (x-z)")], start=2):
        ax = fig.add_subplot(1, 3, idx)
        ax.plot(inicial[:, a], inicial[:, b], color="tab:red", lw=0.8, label="Inicial")
        ax.plot(optimizada[:, a], optimizada[:, b], color="tab:blue", lw=0.8, label="Optimizada")
        ax.set_xlabel("xyz"[a] + " [m]")
        ax.set_ylabel("xyz"[b] + " [m]")
        ax.set_title(nombre)
        ax.axis("equal")
        ax.legend()
        ax.grid(alpha=0.3)

    fig.suptitle(titulo)
    fig.tight_layout()
    _guardar(fig, archivo)
