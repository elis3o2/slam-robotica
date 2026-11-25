"""
Ejercicios 2.2 y 3.2: solución batch (se arma el grafo completo y se optimiza una vez).
"""
import gtsam

from modelo import pose_de_fila, factor_prior, factor_arista


def crear_grafo(poses, edges, dim):
    """Grafo completo: prior en la primera pose + un factor por arista."""
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    # Estimación inicial = poses del archivo (odometría con drift)
    for fila in poses:
        initial.insert(int(fila[0]), pose_de_fila(fila, dim))

    # Prior: fija el sistema de referencia (Gauge Freedom)
    pid0 = int(poses[0, 0])
    graph.add(factor_prior(pid0, pose_de_fila(poses[0], dim), dim))

    # Odometría y cierres de lazo
    for edge in edges:
        graph.add(factor_arista(edge, dim))

    return graph, initial


def gauss_newton(graph, initial, max_iter=100, tolerancia=1e-5):
    """
    Gauss-Newton iterando de a un paso:
      - corta cuando el error relativo cambia menos que `tolerancia`
        (1e-5 es la tolerancia por defecto de GTSAM)
      - si el sistema queda mal condicionado, GTSAM lanza una excepción:
        se atrapa y se devuelve la última solución válida
    Usa QR en vez de Cholesky, que es más estable numéricamente.
    """
    params = gtsam.GaussNewtonParams()
    params.setLinearSolverType("MULTIFRONTAL_QR")
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)

    resultado = initial
    error_previo = graph.error(initial)
    print(f"[Gauss-Newton] error inicial: {error_previo:.3f}")

    for it in range(1, max_iter + 1):
        try:
            optimizer.iterate()
        except RuntimeError:
            print(f"[Gauss-Newton] sistema mal condicionado en la iteración {it}: "
                  f"se usa la última solución válida")
            break

        resultado = optimizer.values()
        error = graph.error(resultado)
        print(f"[Gauss-Newton] iteración {it}: error = {error:.3f}")

        if abs(error_previo - error) < tolerancia * max(error_previo, 1.0):
            break
        error_previo = error

    return resultado


def levenberg_marquardt(graph, initial):
    """Levenberg-Marquardt, para comparar con Gauss-Newton en el informe."""
    params = gtsam.LevenbergMarquardtParams()
    params.setLinearSolverType("MULTIFRONTAL_QR")
    resultado = gtsam.LevenbergMarquardtOptimizer(graph, initial, params).optimize()
    print(f"[Levenberg-Marquardt] error final: {graph.error(resultado):.3f}")
    return resultado
