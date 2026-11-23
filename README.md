# Graph SLAM

Activar el entorno existente:

```bash
source env/bin/activate
```

Si se necesita instalar las dependencias:

```bash
pip install -r requirements.txt
```

Ejecutar desde esta carpeta:

```bash
python ejercicio_2_1.py
python ejercicio_2_2.py
python ejercicio_2_3.py
python ejercicio_3_1.py
python ejercicio_3_2.py
python ejercicio_3_3.py
```

Los ejercicios 2.1 y 3.1 usan los lectores `read_file2d.py` y `read_file3d.py`.
Los ejercicios 2.2 y 3.2 construyen el grafo completo y usan Gauss-Newton.
Los ejercicios 2.3 y 3.3 incorporan las poses y aristas con ISAM2.
Los gráficos se guardan en `resultados/`.

Se usan directamente las poses y mediciones de los archivos, sin perturbación.
La primera pose se ancla con un prior. En 3D se cambia el orden del cuaternión
y de la matriz de información para GTSAM.

La versión incremental está escrita para los datasets de esta práctica:
identificadores consecutivos desde 0 y aristas con origen menor que destino.
La pose nueva se inicializa componiendo una pose ya optimizada con su medición.

En Intel se hacen diez iteraciones explícitas porque el error puede subir en la
primera y bajar después. El informe LaTeX/LyX de la consigna no está incluido.
