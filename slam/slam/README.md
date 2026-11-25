# Graph SLAM con GTSAM

Pose Graph SLAM 2D (dataset INTEL) y 3D (dataset parking-garage), con solución
batch (Gauss-Newton y Levenberg-Marquardt) e incremental (iSAM2).

## Instalación

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Ejecución

Con el entorno activado y los archivos `.g2o` en esta carpeta:

```bash
python slam_2d.py input_INTEL_g2o.g2o
python slam_3d.py parking-garage.g2o
```

Los gráficos se guardan en `resultados/`.

## Estructura

| Archivo | Contenido | Ejercicios |
|---|---|---|
| `lectura_g2o.py` | Lectura de archivos G2O 2D y 3D, armado de la matriz de información | 2.1, 3.1 |
| `modelo.py` | Conversión de los datos a poses y factores de GTSAM (2D o 3D según `dim`) | — |
| `batch.py` | Grafo completo, Gauss-Newton y Levenberg-Marquardt | 2.2, 3.2 |
| `incremental.py` | Solución incremental con iSAM2 | 2.3, 3.3 |
| `graficos.py` | Gráficos de trayectoria inicial vs. optimizada | — |
| `slam_2d.py` | Programa principal 2D | 2.1–2.3 |
| `slam_3d.py` | Programa principal 3D | 3.1–3.3 |

## Decisiones de implementación

- **Prior:** la primera pose se fija con un `PriorFactor` de ruido muy chico (Gauge Freedom).
- **Modelo de ruido:** la matriz de información del archivo se pasa directo con
  `noiseModel.Gaussian.Information`, sin invertirla, porque en estos datasets está muy
  mal condicionada.
- **3D:** el cuaternión se reordena a (w, x, y, z) y se normaliza; los bloques de la
  matriz de información se intercambian porque g2o usa (traslación, rotación) y GTSAM
  (rotación, traslación).
- **Gauss-Newton:** se itera de a un paso hasta que el error relativo cambia menos de
  1e-5 (tolerancia por defecto de GTSAM). Si el sistema queda mal condicionado se
  conserva la última solución válida. Se usa QR en vez de Cholesky.
- **iSAM2:** cada pose nueva se inicializa con la última pose optimizada y se agregan
  las aristas que llegan a ella (odometría y cierres de lazo), agrupadas por nodo.
