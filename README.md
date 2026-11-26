# Graph SLAM

Desde la raíz del proyecto, activar el entorno e instalar las dependencias:

```bash
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt
```

Ejecutar cada solución con:

```bash
python batch_solution2d.py
python incremental_solution2d.py
python batch_solution3d.py
python incremental_solution3d.py
```

Para correr las soluciones **batch** con perturbación de las poses iniciales, agregar `--perturbar`:

```bash
python batch_solution2d.py --perturbar
python batch_solution3d.py --perturbar
```

Las soluciones incrementales se ejecutan sin perturbación. Los datos se leen de `data/` y los gráficos se guardan en `resultados/`; las corridas batch con perturbación generan archivos con el sufijo `_perturbado`.
