"""
Ejercicios 2.3 y 3.3: solución incremental con iSAM2 (Algoritmo 1 del enunciado).
"""
import gtsam

from modelo import pose_de_fila, factor_prior, factor_arista, obtener_pose


def agrupar_aristas_por_nodo(edges):
    """
    {nodo: [aristas que llegan a él]}. Equivale al `if ide2 == idp` del
    Algoritmo 1, pero sin recorrer todas las aristas en cada paso.
    Se usa el mayor de los dos ids por si alguna arista viniera como j < i.
    """
    por_nodo = {}
    for edge in edges:
        clave = int(max(edge[0], edge[1]))
        por_nodo.setdefault(clave, []).append(edge)
    return por_nodo


def isam2(poses, edges, dim):
    aristas_por_nodo = agrupar_aristas_por_nodo(edges)

    params = gtsam.ISAM2Params()
    params.setRelinearizeThreshold(0.1)
    params.relinearizeSkip = 1
    params.setFactorization("QR")       # más estable que Cholesky
    isam = gtsam.ISAM2(params)

    resultado = None
    id_previo = None

    for fila in poses:
        pid = int(fila[0])
        graph = gtsam.NonlinearFactorGraph()   # solo lo nuevo de este paso
        initial = gtsam.Values()

        if id_previo is None:
            # Primera pose: prior + su valor del archivo
            pose = pose_de_fila(fila, dim)
            graph.add(factor_prior(pid, pose, dim))
            initial.insert(pid, pose)
        else:
            # Estimación inicial = última pose ya optimizada
            initial.insert(pid, obtener_pose(resultado, id_previo, dim))

        # Odometría + cierres de lazo que llegan a esta pose
        for edge in aristas_por_nodo.get(pid, []):
            graph.add(factor_arista(edge, dim))

        isam.update(graph, initial)
        resultado = isam.calculateEstimate()
        id_previo = pid

    print(f"[iSAM2] poses optimizadas: {resultado.size()}")
    return resultado
